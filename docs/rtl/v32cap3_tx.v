// =============================================================================
// v32cap3_tx.v —— V32CAP3 数据帧发送器（FPGA 端参考实现）
//
// 功能：
//   连续采 ADC 双通道 12 位样点 -> 缓存一帧 POINTS 点 -> 组帧 -> UART 发出。
//   帧格式（全部小端，除魔数字符串）：见 docs/V32CAP3_FPGA_TX_SPEC.md
//
//   偏移 0   : 8B  魔数 "V32CAP3!" = 56 33 32 43 41 50 33 21
//   偏移 8   : 4B  version        = 3
//   偏移 12  : 4B  frame_id       （每出一帧 +1，可自然回绕）
//   偏移 16  : 4B  adc_fs_hz      = 65000000
//   偏移 20  : 4B  decimation     = 1
//   偏移 24  : 4B  points_per_ch  = 4096
//   偏移 28  : 4B  flags          = 0
//   偏移 32  : 4B  start_index_lo （首点的 ADC 时钟计数低 32 位）
//   偏移 36  : 4B  start_index_hi （高 32 位）
//   偏移 40  : 16384B  4096 组 {CH0 int16, CH1 int16}，组内 CH0 在前
//   偏移 16424: 4B  CRC32，覆盖 [0,16424)，低字节先发
//   总长 16428 B，无帧尾、无分隔符、无换行。
//
// 关键实现约束（不是风格问题，改错就收不到）：
//   1) 12 位有符号样点必须**符号扩展到 16 位**再发，不是零扩展。
//   2) CRC 是"边发边算"，覆盖头部 40 字节 + 全部样点字节，尾部 4 字节不算。
//   3) 一帧发完前不能改写采样 RAM，所以采集与发送是互斥的（本设计用整帧冻结）。
//   4) UART 只有 921600，传一帧 178 ms，而采样一帧只要 63 µs，所以**帧间必然是
//      稀疏抽样，不可能连续**。这是协议物理上限，不是 bug。
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module v32cap3_tx #(
    parameter integer POINTS       = 4096,        // 每通道点数，接收端只认 4096
    parameter integer CLK_FREQ     = 65_000_000,  // clk 频率 = ADC 采样时钟
    parameter integer BAUD         = 921_600,
    parameter integer FS_HZ        = 65_000_000,  // 写进头部的标称采样率
    parameter integer DECIMATION   = 1,
    // 两帧起始点之间的最小间隔（以 clk 计）。0 = 发完立刻采下一帧。
    // 注意：置 0 时实际帧率仍受 UART 限制（约 5.6 fps）。
    parameter integer FRAME_PERIOD = 6_500_000
)(
    input  wire               clk,          // ADC 采样时钟域
    input  wire               rst_n,        // 异步低有效
    input  wire               sample_valid, // 本拍 ch0/ch1 有效（连续采样时接 1'b1）
    input  wire               fault,        // 采集异常（溢出等），当前帧作废
    input  wire signed [11:0] ch0,          // -2048..2047
    input  wire signed [11:0] ch1,
    output wire               tx,           // 接 UART TX 引脚
    output wire               busy,         // 非空闲（正在采集或发送）
    output reg  [31:0]        completed,    // 成功发出的帧数
    output reg  [31:0]        aborted,      // 因 fault 作废的采集次数
    output reg  [31:0]        skipped       // 因 fault 跳过的采样点
);

    localparam integer AW = $clog2(POINTS);

    localparam [2:0] ST_IDLE = 3'd0,
                     ST_CAP  = 3'd1,
                     ST_HDR  = 3'd2,
                     ST_READ = 3'd3,
                     ST_DATA = 3'd4,
                     ST_TAIL = 3'd5;

    // ---- 魔数字节（按发送顺序） ----
    localparam [7:0] M0 = 8'h56, M1 = 8'h33, M2 = 8'h32, M3 = 8'h43,
                     M4 = 8'h41, M5 = 8'h50, M6 = 8'h33, M7 = 8'h21;

    reg [2:0]    state;
    reg [31:0]   timer;
    reg [31:0]   frame_id;
    reg [AW-1:0] addr;
    reg [5:0]    pos;           // 头部 0..39 / 样点组 0..3 / 尾部 0..3
    reg [31:0]   crc;
    reg [31:0]   word_q;        // 当前 32 位样点字 {CH1, CH0}

    reg [63:0]   sample_index;  // 复位起的 ADC 时钟计数
    reg [63:0]   start_index;   // 本帧首点的计数

    // 采样缓存：一个 32 位字 = {CH1[15:0], CH0[15:0]}
    (* ram_style = "block" *) reg [31:0] memory [0:POINTS-1];

    // ---- UART ----
    reg  [7:0] byte_data;
    wire       tx_ready;
    wire       sending = (state == ST_HDR) || (state == ST_DATA) || (state == ST_TAIL);

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (byte_data),
        .tx_valid (sending),
        .tx_ready (tx_ready),
        .tx       (tx)
    );

    assign busy = (state != ST_IDLE);

    // ---- 样点打包：12 位有符号 -> 16 位有符号（符号扩展，必须） ----
    wire [31:0] packed_sample = {{4{ch1[11]}}, ch1, {4{ch0[11]}}, ch0};

    wire        begin_capture = (state == ST_IDLE) && (timer == 32'd0) && sample_valid && !fault;
    wire        memory_we     = begin_capture || ((state == ST_CAP) && sample_valid && !fault);
    wire [AW-1:0] memory_wr_addr = begin_capture ? {AW{1'b0}} : addr;

    always @(posedge clk) begin
        // 单写口 RAM：首点在 begin_capture 那一拍写地址 0，其余按 addr 递增
        if (memory_we) memory[memory_wr_addr] <= packed_sample;
        if (state == ST_READ) word_q <= memory[addr];
    end

    // ---- CRC32：反射多项式 0xEDB88320，逐字节 ----
    function [31:0] crc_byte;
        input [31:0] c;
        input [7:0]  d;
        integer      k;
        reg   [31:0] v;
        begin
            v = c ^ {24'd0, d};
            for (k = 0; k < 8; k = k + 1)
                v = v[0] ? ((v >> 1) ^ 32'hEDB8_8320) : (v >> 1);
            crc_byte = v;
        end
    endfunction

    // ---- 头部 8 个 u32 的选择（pos[5:2] = pos/4 - 2） ----
    reg [31:0] header_word;
    always @* begin
        case (pos[5:2])
            4'd2:    header_word = 32'd3;                 // version
            4'd3:    header_word = frame_id;              // frame_id
            4'd4:    header_word = FS_HZ;                 // adc_fs_hz
            4'd5:    header_word = DECIMATION;            // decimation
            4'd6:    header_word = POINTS;                // points_per_channel
            4'd7:    header_word = 32'd0;                 // flags，固定 0
            4'd8:    header_word = start_index[31:0];     // start_index_lo
            4'd9:    header_word = start_index[63:32];    // start_index_hi
            default: header_word = 32'd0;
        endcase
    end

    // ---- 当前要发的那个字节（组合逻辑，pos/state 决定） ----
    always @* begin
        byte_data = 8'h00;
        if (state == ST_HDR) begin
            if (pos < 6'd8) begin
                case (pos[2:0])
                    3'd0: byte_data = M0;
                    3'd1: byte_data = M1;
                    3'd2: byte_data = M2;
                    3'd3: byte_data = M3;
                    3'd4: byte_data = M4;
                    3'd5: byte_data = M5;
                    3'd6: byte_data = M6;
                    3'd7: byte_data = M7;
                endcase
            end else begin
                byte_data = header_word >> (8 * pos[1:0]);   // 小端：低字节先发
            end
        end else if (state == ST_DATA) begin
            byte_data = word_q >> (8 * pos[1:0]);            // CH0 低/高, CH1 低/高
        end else if (state == ST_TAIL) begin
            byte_data = (~crc) >> (8 * pos[1:0]);            // 最终异或 FFFFFFFF
        end
    end

    // ---- 主状态机 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            timer        <= 32'd0;
            frame_id     <= 32'd0;
            sample_index <= 64'd0;
            start_index  <= 64'd0;
            addr         <= {AW{1'b0}};
            pos          <= 6'd0;
            crc          <= 32'hFFFF_FFFF;
            completed    <= 32'd0;
            aborted      <= 32'd0;
            skipped      <= 32'd0;
        end else begin
            // ADC 时钟计数：帧间死区也照走，这样 start_index 能反映真实时间
            sample_index <= sample_index + 64'd1;

            if (timer != 32'd0) timer <= timer - 32'd1;

            case (state)
                ST_IDLE: begin
                    if (begin_capture) begin
                        start_index <= sample_index;
                        addr        <= {{(AW-1){1'b0}}, 1'b1};
                        state       <= ST_CAP;
                        timer       <= FRAME_PERIOD;
                    end else if (fault && sample_valid) begin
                        skipped <= skipped + 32'd1;
                    end
                end

                ST_CAP: begin
                    if (fault || !sample_valid) begin
                        state   <= ST_IDLE;
                        aborted <= aborted + 32'd1;
                        timer   <= FRAME_PERIOD;
                    end else if (addr == POINTS-1) begin
                        frame_id <= frame_id + 32'd1;
                        state    <= ST_HDR;
                        pos      <= 6'd0;
                        crc      <= 32'hFFFF_FFFF;
                        addr     <= {AW{1'b0}};
                    end else begin
                        addr <= addr + {{(AW-1){1'b0}}, 1'b1};
                    end
                end

                ST_HDR: begin
                    if (tx_ready) begin
                        crc <= crc_byte(crc, byte_data);
                        if (pos == 6'd39) begin
                            state <= ST_READ;
                            pos   <= 6'd0;
                        end else begin
                            pos <= pos + 6'd1;
                        end
                    end
                end

                ST_READ: begin
                    // 取一个样点字，空一拍给 block RAM 读出
                    state <= ST_DATA;
                    pos   <= 6'd0;
                end

                ST_DATA: begin
                    if (tx_ready) begin
                        crc <= crc_byte(crc, byte_data);
                        if (pos == 6'd3) begin
                            pos <= 6'd0;
                            if (addr == POINTS-1) begin
                                state <= ST_TAIL;
                            end else begin
                                addr  <= addr + {{(AW-1){1'b0}}, 1'b1};
                                state <= ST_READ;
                            end
                        end else begin
                            pos <= pos + 6'd1;
                        end
                    end
                end

                ST_TAIL: begin
                    if (tx_ready) begin
                        if (pos == 6'd3) begin
                            pos       <= 6'd0;
                            state     <= ST_IDLE;
                            completed <= completed + 32'd1;
                        end else begin
                            pos <= pos + 6'd1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
