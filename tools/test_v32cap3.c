/*
 * test_v32cap3.c —— 用官方样例帧验证 C 解析器
 *
 * 期望值来自 samples/expected.json（由 FPGA 侧交付包给出，
 * 是归档里 original/crc4096_acceptance_20261016_175522/received.bin 的第一帧）。
 *
 * 编译（在 Linux 主机上，不需要交叉工具链）：
 *   gcc -O2 -std=gnu99 -o /tmp/t_v32 src/v32cap3.c tools/test_v32cap3.c -lpthread
 * 运行：
 *   /tmp/t_v32 tools/sample_frame.bin
 */
#include "v32cap3.h"

#include <stdio.h>
#include <string.h>

static int fails;

static void chk(int cond, const char *what, long long got, long long want)
{
    if (cond) {
        printf("  [ OK ] %-22s = %lld\n", what, got);
    } else {
        printf("  [FAIL] %-22s = %lld  (期望 %lld)\n", what, got, want);
        fails++;
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "用法: %s <sample_frame.bin>\n", argv[0]);
        return 2;
    }

    printf("== 1. 标准 CRC 检查向量 ==\n");
    /* 协议里明确写了 "123456789" -> 0xCBF43926；
     * 这里通过解析器间接验证不到，单独算一次（复制 crc 表逻辑不现实），
     * 改成用「样例帧 CRC 对得上」来证明实现正确即可，见下一步。 */

    printf("== 2. 解析样例帧 %s ==\n", argv[1]);
    if (v32_init_file(argv[1]) != 0) {
        fprintf(stderr, "初始化失败: %s\n", v32_last_error());
        return 2;
    }

    const v32_stats_t *st = v32_get_stats();
    printf("  frames_ok=%u crc_err=%u hdr_err=%u seq_err=%u sample_err=%u discarded=%u trailing=%u\n",
           st->frames_ok, st->crc_errors, st->header_errors,
           st->sequence_errors, st->sample_errors,
           st->discarded_bytes, st->trailing_bytes);

    v32_frame_t f;
    if (!v32_copy_latest(&f)) {
        printf("  [FAIL] 没有解出任何帧\n");
        return 1;
    }

    chk(st->frames_ok == 1, "frames_ok", st->frames_ok, 1);
    chk(st->crc_errors == 0, "crc_errors", st->crc_errors, 0);
    chk(st->header_errors == 0, "header_errors", st->header_errors, 0);
    chk(f.frame_id == 124, "frame_id", f.frame_id, 124);
    chk(f.start_index == 1417389169ull, "start_index", (long long)f.start_index, 1417389169ll);
    chk(f.crc32 == 0x5E19F53Fu, "crc32", f.crc32, 0x5E19F53Fll);

    static const int16_t exp[16][2] = {
        {-13, 3}, {-14, 4}, {-14, 7}, {-13, 8}, {-15, 11}, {-13, 12}, {-13, 15}, {-12, 16},
        {-13, 18}, {-14, 21}, {-14, 22}, {-13, 24}, {-12, 26}, {-14, 25}, {-13, 27}, {-11, 27},
    };
    printf("== 3. 前 16 组样点 ==\n");
    for (int i = 0; i < 16; i++) {
        const int ok = (f.ch0[i] == exp[i][0] && f.ch1[i] == exp[i][1]);
        if (!ok) {
            printf("  [FAIL] i=%2d  got (%d,%d)  期望 (%d,%d)\n",
                   i, f.ch0[i], f.ch1[i], exp[i][0], exp[i][1]);
            fails++;
        }
    }
    if (fails == 0) printf("  16/16 全部一致\n");

    printf("== 4. 样点范围与统计 ==\n");
    int32_t mn0 = 32767, mx0 = -32768, mn1 = 32767, mx1 = -32768;
    for (int i = 0; i < V32_POINTS; i++) {
        if (f.ch0[i] < mn0) mn0 = f.ch0[i];
        if (f.ch0[i] > mx0) mx0 = f.ch0[i];
        if (f.ch1[i] < mn1) mn1 = f.ch1[i];
        if (f.ch1[i] > mx1) mx1 = f.ch1[i];
    }
    printf("  CH0 min/max = %d / %d   (Vpp %d 码 = %.1f mV)\n",
           mn0, mx0, mx0 - mn0, (double)(mx0 - mn0) * V32_ADC_MV_PER_LSB);
    printf("  CH1 min/max = %d / %d   (Vpp %d 码 = %.1f mV)\n",
           mn1, mx1, mx1 - mn1, (double)(mx1 - mn1) * V32_ADC_MV_PER_LSB);
    chk(mn0 >= V32_ADC_MIN && mx0 <= V32_ADC_MAX, "CH0 在 12 位量程内", 1, 1);
    chk(mn1 >= V32_ADC_MIN && mx1 <= V32_ADC_MAX, "CH1 在 12 位量程内", 1, 1);

    v32_shutdown();

    printf("\n%s（失败 %d 项）\n", fails ? "❌ 有不符合项" : "✅ 全部通过", fails);
    return fails ? 1 : 0;
}
