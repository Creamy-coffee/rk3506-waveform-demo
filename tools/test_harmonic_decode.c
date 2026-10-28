/*
 * test_harmonic_decode.c —— 谐波帧 V32HRM1! 的主机端回归
 *
 * 目的：在**没有板子、没有串口**的情况下，把交付包文档要求验收的几条
 * 先钉死：
 *   1) v2 正常包能解出来，THD 用整数开方算，和测试里独立算的期望值一致；
 *   2) v2 线上 thd_q24 字段恒为 0，绝不能当 0% 失真 —— 解出来的 q24 必须
 *      是按 signal_x6 算出来的非零值；
 *   3) v1 / v2 靠 flags bit9 区分，bit9 与 version 不符的包必须拒绝；
 *   4) 坏 CRC / 长度不符 / noise_mask 不是 valid_mask 子集 必须拒绝；
 *   5) H2..H5 全部低于噪声时 thd_valid 必须被撤回、thd_below_noise=1；
 *   6) 走真实分发器（v32_test_feed_bytes）时，碎片输入、前导垃圾、截断包
 *      后面紧跟好包，都要能拼回来 / 重新同步，且只接受真正有效的那一包。
 *
 * 这些都只是**软件回归**：不等于 ARM 已编译通过，更不等于板上实测通过。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "harmonic_receiver.h"
#include "v32cap3.h"

static int g_fail;
static int g_pass;

#define CHECK(cond, fmt, ...) do {                                  \
    if (cond) { g_pass++; }                                         \
    else { g_fail++; printf("  FAIL  " fmt "\n", ##__VA_ARGS__); }   \
} while (0)

/* ---------- 组包工具（与协议表逐字节对齐） ---------- */
static uint32_t crc32_of(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ ((c & 1) ? 0xEDB88320u : 0);
    }
    return ~c;
}
static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void wr64(uint8_t *p, uint64_t v)
{
    wr32(p, (uint32_t)v); wr32(p + 4, (uint32_t)(v >> 32));
}

/* 期望 THD（和接收器里的整数算法同源，但独立写一遍以交叉校验） */
static uint64_t isqrt64(uint64_t x)
{
    uint64_t r = 0, bit = (uint64_t)1 << 62;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

typedef struct {
    uint32_t version, rid, cfg, adc_fs, nfft, fcw, dec, age, flags;
    uint32_t ch, win, valid_mask, noise_mask, thd_field, dropped;
    uint64_t band[5], noise[5], sig[5];
    uint32_t peak[5];
} pkt_spec_t;

static void build_packet(const pkt_spec_t *s, uint8_t out[V32_HRM_BYTES])
{
    memset(out, 0, V32_HRM_BYTES);
    memcpy(out, "V32HRM1!", 8);
    wr32(out + 8,  s->version);
    wr32(out + 12, V32_HRM_BYTES);
    wr32(out + 16, s->rid);
    wr32(out + 20, s->cfg);
    wr32(out + 24, s->adc_fs);
    wr32(out + 28, s->nfft);
    wr32(out + 32, s->fcw);
    wr32(out + 36, s->dec);
    wr32(out + 40, s->age);
    wr32(out + 44, s->flags);
    wr32(out + 48, s->ch);
    wr32(out + 52, s->win);
    wr32(out + 56, s->valid_mask);
    wr32(out + 60, s->noise_mask);
    wr32(out + 64, s->thd_field);
    wr32(out + 68, s->dropped);
    for (int i = 0; i < 5; i++) {
        wr64(out + 72  + 8 * i, s->band[i]);
        wr64(out + 112 + 8 * i, s->noise[i]);
        wr64(out + 152 + 8 * i, s->sig[i]);
        wr32(out + 192 + 4 * i, s->peak[i]);
    }
    wr32(out + 212, crc32_of(out, 212));
}

/* 一组合法的 v2 底稿：基波 x6=6000000，二~五阶 x6 共 60000 → THD≈10% */
static void base_v2(pkt_spec_t *s, uint32_t rid)
{
    memset(s, 0, sizeof *s);
    s->version = 2u;
    s->rid = rid;
    s->cfg = 77u;
    s->adc_fs = 65000000u;
    s->nfft = 4096u;
    s->fcw = 66067367u;          /* ≈1 MHz 基频参考 */
    s->dec = 16u;
    s->age = 4096u;
    s->flags = 0x80000000u | 0x200u;   /* bit31 统计资格 + bit9 主机计算 */
    s->ch = 0u;
    s->win = 0u;                  /* Hann */
    s->valid_mask = 31u;
    s->noise_mask = 0u;
    s->thd_field = 0u;            /* v2 线上必须为 0 */
    s->dropped = 0u;
    static const uint64_t sig[5] = { 6000000u, 6000u, 12000u, 18000u, 24000u };
    for (int i = 0; i < 5; i++) {
        s->sig[i] = sig[i];
        s->noise[i] = 0u;
        s->band[i] = sig[i] / 6u;   /* noise=0 → signal = 6*band */
        s->peak[i] = (uint32_t)(128 * (i + 1));
    }
}

int main(void)
{
    uint8_t pkt[V32_HRM_BYTES];
    v32_harmonic_measurement m;
    pkt_spec_t s;

    printf("== 1. v2 正常包 ==\n");
    base_v2(&s, 1001u);
    build_packet(&s, pkt);
    memset(&m, 0, sizeof m);
    int ok = v32_harmonic_decode(pkt, V32_HRM_BYTES, &m);
    CHECK(ok == 1, "v2 正常包应被接受，实际 %d", ok);
    const uint64_t sum = s.sig[1] + s.sig[2] + s.sig[3] + s.sig[4];
    const uint64_t want_q = isqrt64((sum << 48) / s.sig[0]);
    CHECK(m.thd_q24 == (uint32_t)want_q, "thd_q24 应为 %llu，实际 %u",
          (unsigned long long)want_q, m.thd_q24);
    CHECK(m.thd_valid == 1, "thd_valid 应为 1，实际 %d", m.thd_valid);
    CHECK(m.thd_below_noise == 0, "thd_below_noise 应为 0，实际 %d", m.thd_below_noise);
    CHECK(m.fpga_thd_q24 == 0u, "线上字段 fpga_thd_q24 应为 0（v2 恒 0），实际 %u", m.fpga_thd_q24);
    /* 关键红线：线上 0 不能当 0% 失真 —— 主机算出来的必须是 ~10% */
    const double thd_pct = (double)m.thd_q24 * 100.0 / 16777216.0;
    CHECK(thd_pct > 9.9 && thd_pct < 10.1, "THD 应约 10%%，实际 %.4f%%", thd_pct);
    printf("     rid=%u thd_q24=%u → %.4f%% (期望 %.4f%%)\n",
           m.result_id, m.thd_q24, thd_pct, 100.0 * (double)want_q / 16777216.0);

    printf("== 2. dBc 换算 ==\n");
    double dbc = 0.0;
    CHECK(v32_harmonic_dbc(&m, 1u, &dbc) == 1, "H1 dBc 应可算");
    CHECK(dbc > -0.001 && dbc < 0.001, "H1 相对自己应为 0 dBc，实际 %.6f", dbc);
    CHECK(v32_harmonic_dbc(&m, 2u, &dbc) == 1, "H2 dBc 应可算");
    /* 6000/6000000 = 1e-3 → -30 dBc */
    CHECK(dbc > -30.01 && dbc < -29.99, "H2 应约 -30 dBc，实际 %.4f", dbc);
    CHECK(v32_harmonic_dbc(&m, 6u, &dbc) == 0, "6 阶应拒绝（只支持 2..5 阶）");

    printf("== 3. 坏 CRC ==\n");
    base_v2(&s, 1002u);
    build_packet(&s, pkt);
    pkt[100] ^= 0xFFu;
    CHECK(v32_harmonic_decode(pkt, V32_HRM_BYTES, &m) == 0, "坏 CRC 必须拒绝");

    printf("== 4. 长度不符 ==\n");
    base_v2(&s, 1003u);
    build_packet(&s, pkt);
    wr32(pkt + 12, 200u);                       /* 头里长度写错 */
    wr32(pkt + 212, crc32_of(pkt, 212));        /* CRC 重算，只坏长度 */
    CHECK(v32_harmonic_decode(pkt, V32_HRM_BYTES, &m) == 0, "length!=216 必须拒绝");

    printf("== 5. v1 / v2 靠 bit9 区分 ==\n");
    base_v2(&s, 1004u);
    s.flags = 0x80000000u;                      /* v2 却清掉 bit9 */
    build_packet(&s, pkt);
    CHECK(v32_harmonic_decode(pkt, V32_HRM_BYTES, &m) == 0, "v2 缺 bit9 必须拒绝");

    s.version = 1u;
    s.flags = 0x80000000u;                      /* v1 必须清 bit9 */
    s.thd_field = (uint32_t)want_q;             /* v1 由 FPGA 直接给 THD */
    build_packet(&s, pkt);
    ok = v32_harmonic_decode(pkt, V32_HRM_BYTES, &m);
    CHECK(ok == 1, "v1 自洽包应被接受，实际 %d", ok);
    CHECK(m.computed_on_host == 0, "v1 的 computed_on_host 应为 0");
    CHECK(m.fpga_thd_q24 == (uint32_t)want_q, "v1 应保留 FPGA 给的 THD");

    s.thd_field = (uint32_t)want_q + 1u;        /* v1 的 THD 与功率不一致 */
    build_packet(&s, pkt);
    CHECK(v32_harmonic_decode(pkt, V32_HRM_BYTES, &m) == 0,
          "v1 的 thd_q24 与功率不自洽必须拒绝");

    printf("== 6. noise_mask 必须是 valid_mask 子集 ==\n");
    base_v2(&s, 1005u);
    s.valid_mask = 0x01u;                       /* 只有 H1 有效 */
    s.noise_mask = 0x02u;                       /* 却声称 H2 低于噪声 */
    build_packet(&s, pkt);
    CHECK(v32_harmonic_decode(pkt, V32_HRM_BYTES, &m) == 0, "noise⊄valid 必须拒绝");

    printf("== 7. H2..H5 全部低于噪声 → 撤回正式 THD ==\n");
    base_v2(&s, 1006u);
    s.noise_mask = 0x1Eu;                       /* H2..H5 低于噪声，H1 不是 */
    build_packet(&s, pkt);
    ok = v32_harmonic_decode(pkt, V32_HRM_BYTES, &m);
    CHECK(ok == 1, "低于噪声的包本身是合法包，应被接受，实际 %d", ok);
    CHECK(m.thd_below_noise == 1, "thd_below_noise 应为 1，实际 %d", m.thd_below_noise);
    CHECK(m.thd_valid == 0, "低于噪声时 thd_valid 必须为 0（不能显示 0%% 失真）");
    CHECK(v32_harmonic_dbc(&m, 3u, &dbc) == 0, "低于噪声的阶次不给 dBc");

    printf("== 8. 分发器：碎片拼接（逐字节喂） ==\n");
    base_v2(&s, 1007u);
    build_packet(&s, pkt);
    const uint32_t rx0 = v32_harm_rx_count();
    for (int i = 0; i < V32_HRM_BYTES; i++) v32_test_feed_bytes(pkt + i, 1);
    CHECK(v32_harm_rx_count() == rx0 + 1u, "逐字节喂应恰好接受 1 包，实际 +%u",
          v32_harm_rx_count() - rx0);
    uint32_t age = 0;
    CHECK(v32_get_harm(&m, &age) == 1 && m.result_id == 1007u,
          "逐字节喂后应取到 rid=1007，实际 %u", m.result_id);

    printf("== 9. 分发器：前导垃圾 + 3 段碎片 ==\n");
    base_v2(&s, 1008u);
    build_packet(&s, pkt);
    uint8_t junk[37];
    for (int i = 0; i < (int)sizeof junk; i++) junk[i] = (uint8_t)(0x40 + i);
    const uint32_t rx1 = v32_harm_rx_count();
    v32_test_feed_bytes(junk, (int)sizeof junk);
    v32_test_feed_bytes(pkt, 61);
    v32_test_feed_bytes(pkt + 61, 62);
    v32_test_feed_bytes(pkt + 123, V32_HRM_BYTES - 123);
    CHECK(v32_harm_rx_count() == rx1 + 1u, "垃圾+碎片应接受 1 包，实际 +%u",
          v32_harm_rx_count() - rx1);
    CHECK(v32_get_harm(&m, &age) == 1 && m.result_id == 1008u,
          "应取到 rid=1008，实际 %u", m.result_id);

    printf("== 10. 分发器：坏 CRC 后紧跟好包 → 重同步 ==\n");
    base_v2(&s, 1009u);
    build_packet(&s, pkt);
    pkt[150] ^= 0x5Au;                          /* 先坏一包 */
    uint8_t good[V32_HRM_BYTES];
    base_v2(&s, 1010u);
    build_packet(&s, good);
    const uint32_t rx2 = v32_harm_rx_count();
    v32_test_feed_bytes(pkt, V32_HRM_BYTES);
    v32_test_feed_bytes(good, V32_HRM_BYTES);
    CHECK(v32_harm_rx_count() == rx2 + 1u, "坏包应被丢、好包应收下（净 +1），实际 +%u",
          v32_harm_rx_count() - rx2);
    CHECK(v32_get_harm(&m, &age) == 1 && m.result_id == 1010u,
          "重同步后应取到 rid=1010，实际 %u", m.result_id);

    printf("== 11. 分发器：截断包后紧跟好包 → 重同步 ==\n");
    base_v2(&s, 1011u);
    build_packet(&s, pkt);
    base_v2(&s, 1012u);
    build_packet(&s, good);
    const uint32_t rx3 = v32_harm_rx_count();
    v32_test_feed_bytes(pkt, 100);              /* 只发前 100 字节就断 */
    v32_test_feed_bytes(good, V32_HRM_BYTES);
    CHECK(v32_harm_rx_count() == rx3 + 1u, "截断包应被丢、好包应收下，实际 +%u",
          v32_harm_rx_count() - rx3);
    CHECK(v32_get_harm(&m, &age) == 1 && m.result_id == 1012u,
          "应取到 rid=1012，实际 %u", m.result_id);

    printf("== 12. 新鲜度：重复 result_id 不刷新 ==\n");
    base_v2(&s, 1013u);
    build_packet(&s, pkt);
    for (int k = 0; k < 3; k++) v32_test_feed_bytes(pkt, V32_HRM_BYTES);
    uint32_t age_a = 0;
    CHECK(v32_get_harm(&m, &age_a) == 1, "应能取到结果");
    CHECK(v32_harm_rx_count() >= 3u, "重复包应都被计入收包数（诊断用）");
    /* 同一 rid 连发 3 次后，年龄仍从第一次到达算，不会因为重发归零 */
    CHECK(age_a < 1000u, "年龄应为毫秒级（%u），不该被重发放大", age_a);

    printf("== 13. DDC/ANA 与谐波互不干扰 ==\n");
    /* 谐波包夹在 CAP3/ANA 之外时，不应污染 ANA 的 cfg 状态机 —— 这里只验证
     * 谐波包不会把 stats 的帧计数当成波形帧（frames_ok 不变）。 */
    const uint32_t frames_before = v32_get_stats()->frames_ok;
    base_v2(&s, 1014u);
    build_packet(&s, pkt);
    v32_test_feed_bytes(pkt, V32_HRM_BYTES);
    CHECK(v32_get_stats()->frames_ok == frames_before,
          "谐波包不应被算成 V32CAP3 波形帧，frames_ok 从 %u 变成了 %u",
          frames_before, v32_get_stats()->frames_ok);

    printf("\n结果：%d 通过 / %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
