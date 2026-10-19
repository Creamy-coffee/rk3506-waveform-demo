// =============================================================================
// uart_tx.v —— 8N1 UART 发送器（V32CAP3 配套参考实现）
//
// 为什么单独放一份而不是用通用 IP：
//   1) 分频系数必须**四舍五入**而不是直接整除截断，否则 65 MHz 下
//      65000000/921600 = 70.53，截断成 70 只有 +0.76% 误差（勉强），
//      而某些时钟会超过 1%，长时间累积可能丢字符。
//   2) 接口统一为 valid/ready 握手，高位在前（LSB first）由内部处理。
//
// 波特率误差参考（921600 目标）：
//   24.0 MHz -> 26 分频, 923077 Bd, +0.16%
//   50.0 MHz -> 54 分频, 925926 Bd, +0.47%
//   65.0 MHz -> 71 分频, 915493 Bd, -0.66%   <= 本项目 65 MHz 时钟
//   100  MHz -> 109 分频, 917431 Bd, -0.45%
//   125  MHz -> 136 分频, 919118 Bd, -0.27%
//   200  MHz -> 217 分频, 921659 Bd, +0.006%
//
// 握手语义：
//   tx_valid && tx_ready 在同一拍成立 -> 该字节被接收，下一拍 tx 开始拉低发起始位。
//   tx_ready 只在空闲态为高，因此不要依赖它做背压，直接看 valid/ready 同时为高即可。
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_tx #(
    parameter integer CLK_FREQ = 65_000_000,
    parameter integer BAUD     = 921_600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    output reg        tx
);

    // 四舍五入，避免整除截断带来的系统性偏差
    localparam integer CLKS_PER_BIT = (CLK_FREQ + BAUD/2) / BAUD;
    localparam integer CNT_W        = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT);

    localparam [1:0] ST_IDLE = 2'd0,
                     ST_START= 2'd1,
                     ST_DATA = 2'd2,
                     ST_STOP = 2'd3;

    reg [1:0]       state;
    reg [CNT_W-1:0] clk_count;
    reg [2:0]       bit_index;
    reg [7:0]       data_latched;

    assign tx_ready = (state == ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            clk_count    <= {CNT_W{1'b0}};
            bit_index    <= 3'd0;
            data_latched <= 8'd0;
            tx           <= 1'b1;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx        <= 1'b1;
                    clk_count <= {CNT_W{1'b0}};
                    bit_index <= 3'd0;
                    if (tx_valid) begin
                        data_latched <= tx_data;
                        tx           <= 1'b0;          // 起始位
                        state        <= ST_START;
                    end
                end

                ST_START: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= {CNT_W{1'b0}};
                        tx        <= data_latched[0];  // LSB first
                        bit_index <= 3'd0;
                        state     <= ST_DATA;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= {CNT_W{1'b0}};
                        if (bit_index == 3'd7) begin
                            tx    <= 1'b1;             // 停止位
                            state <= ST_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx        <= data_latched[bit_index + 1'b1];
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= {CNT_W{1'b0}};
                        tx        <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    tx    <= 1'b1;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
