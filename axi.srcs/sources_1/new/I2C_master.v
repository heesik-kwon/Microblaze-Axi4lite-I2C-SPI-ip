module I2C_Master (
    // global signals
    input        clk,
    input        reset,
    // AXI4 reg signals
    input  [7:0] tx_data,
    output [7:0] rx_data,
    output       done,
    output       ready,
    input  [1:0] mode,     // 00: send data, 01:send start, 10: send stop, 11: read data
    input        enable,
    // I2C signals
    output       SCL,
    inout        SDA       // 
);

    localparam  IDLE   = 0,  HOLD = 1,
                START1 = 2,  START2 = 3,
                WDATA1 = 4,  WDATA2 = 5, WDATA3 =6,   WDATA4 = 7, 
                RDATA1 = 8,  RDATA2 = 9, RDATA3 = 10, RDATA4 = 11, 
                ACK1   = 12, ACK2   = 13,ACK3   = 14, ACK4   = 15,
                SEND_ACK1   = 16, SEND_ACK2   = 17,SEND_ACK3   = 18, SEND_ACK4   = 19,
                STOP1  = 20, STOP2  = 21;

    reg [4:0] state, state_next;
    reg [7:0] tx_data_reg, tx_data_next;
    reg [7:0] rx_data_reg, rx_data_next;
    reg [3:0] bit_cnt_reg, bit_cnt_next;
    reg [$clog2(500)-1:0] clk_cnt_reg, clk_cnt_next;
    reg scl, scl_next;
    reg done_reg, done_next;
    reg ready_reg, ready_next;

    assign SCL = scl;
    assign SDA = ( (state == ACK1) | (state == ACK2) | (state == ACK3) | (state == ACK4)) ? 1'bz :
                 ( (state == RDATA1) | (state == RDATA2) | (state == RDATA3) | (state == RDATA4)) ? 1'bz : tx_data_reg[7];
    assign done = done_reg;
    assign ready = ready_reg;
    assign rx_data = rx_data_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            tx_data_reg <= 8'hFF;
            rx_data_reg <= 8'b0;
            bit_cnt_reg <= 4'b0;
            clk_cnt_reg <= 0;
            scl <= 1'b1;
            done_reg <= 0;
            ready_reg <= 1;
        end else begin
            state <= state_next;
            tx_data_reg <= tx_data_next;
            rx_data_reg <= rx_data_next;
            bit_cnt_reg <= bit_cnt_next;
            clk_cnt_reg <= clk_cnt_next;
            scl <= scl_next;
            done_reg <= done_next;
            ready_reg <= ready_next;
        end
    end

    always @(*) begin
        state_next = state;
        tx_data_next = tx_data_reg;
        rx_data_next = rx_data_reg;
        bit_cnt_next = bit_cnt_reg;
        clk_cnt_next = clk_cnt_reg;
        scl_next = scl;
        done_next = done_reg;
        ready_next = ready_reg;
        case (state)
            IDLE: begin
                scl_next        = 1'b1;
                tx_data_next[7] = 1'b1;
                done_next       = 1'b0;
                ready_next      = 1'b1;
                if (enable) begin
                    case (mode)
                        2'b00: begin
                            state_next   = WDATA1;  // SEND TX
                            tx_data_next = tx_data;
                            scl_next     = 1'b0;
                            bit_cnt_next = 1'b0;
                            clk_cnt_next = 1'b0;
                            ready_next   = 1'b0;
                        end
                        2'b01: begin
                            state_next      = START1;  // START
                            scl_next        = 1'b1;
                            tx_data_next[7] = 1'b0;
                            clk_cnt_next    = 1'b0;
                            ready_next      = 1'b0;
                        end
                        2'b10: begin
                            state_next      = STOP1;  // STOP
                            scl_next        = 1'b1;
                            tx_data_next[7] = 1'b0;
                            clk_cnt_next    = 1'b0;
                            ready_next      = 1'b0;
                        end
                        2'b11: begin
                            state_next   = RDATA1;  // READ RX
                            scl_next     = 1'b0;
                            bit_cnt_next = 1'b0;
                            clk_cnt_next = 1'b0;
                            ready_next   = 1'b0;
                        end
                    endcase
                end
            end
            HOLD: begin
                scl_next = 1'b0;
                tx_data_next[7] = 1'b0;
                done_next = 1'b0;
                ready_next = 1'b1;
                if (enable) begin
                    case (mode)
                        2'b00: begin
                            state_next   = WDATA1;  // SEND TX
                            tx_data_next = tx_data;
                            scl_next     = 1'b0;
                            bit_cnt_next = 1'b0;
                            clk_cnt_next = 1'b0;
                            ready_next   = 1'b0;
                        end
                        2'b01: begin
                            state_next      = START1;  // START
                            scl_next        = 1'b1;
                            tx_data_next[7] = 1'b0;
                            clk_cnt_next    = 1'b0;
                            ready_next      = 1'b0;
                        end
                        2'b10: begin
                            state_next      = STOP1;  // STOP
                            scl_next        = 1'b1;
                            tx_data_next[7] = 1'b0;
                            clk_cnt_next    = 1'b0;
                            ready_next      = 1'b0;
                        end
                        2'b11: begin
                            state_next   = RDATA1;  // READ RX
                            scl_next     = 1'b0;
                            bit_cnt_next = 1'b0;
                            clk_cnt_next = 1'b0;
                            ready_next   = 1'b0;
                        end
                    endcase
                end
            end
            START1: begin
                scl_next        = 1'b1;
                tx_data_next[7] = 1'b0;
                if (clk_cnt_reg == 499) begin
                    state_next      = START2;
                    clk_cnt_next    = 0;
                    scl_next        = 1'b0;
                    tx_data_next[7] = 1'b0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            START2: begin
                scl_next        = 1'b0;
                tx_data_next[7] = 1'b0;
                if (clk_cnt_reg == 499) begin
                    state_next      = HOLD;
                    clk_cnt_next    = 0;
                    scl_next        = 1'b0;
                    tx_data_next[7] = 1'b0;
                    done_next       = 1'b1;
                    ready_next      = 1'b1;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            WDATA1: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    state_next   = WDATA2;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            WDATA2: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = WDATA3;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            WDATA3: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = WDATA4;
                    scl_next     = 0;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            WDATA4: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    clk_cnt_next = 0;
                    scl_next     = 0;
                    if (bit_cnt_reg == 7) begin
                        state_next      = ACK1;
                        tx_data_next[7] = 1'b0;
                        bit_cnt_next    = 0;
                    end else begin
                        state_next   = WDATA1;
                        tx_data_next = {tx_data_reg[6:0], 1'b0};
                        bit_cnt_next = bit_cnt_reg + 1;
                    end
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            RDATA1: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    state_next   = RDATA2;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            RDATA2: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = RDATA3;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                    rx_data_next = {rx_data_reg[6:0], SDA};
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            RDATA3: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = RDATA4;
                    scl_next     = 0;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            RDATA4: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    clk_cnt_next = 0;
                    scl_next     = 0;
                    if (bit_cnt_reg == 7) begin
                        state_next = SEND_ACK1;
                        tx_data_next[7] = 1'b0;
                        bit_cnt_next = 0;
                    end else begin
                        state_next   = RDATA1;
                        bit_cnt_next = bit_cnt_reg + 1;
                    end
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            ACK1: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    state_next   = ACK2;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            ACK2: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = ACK3;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                    rx_data_next = {7'b0, SDA};
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            ACK3: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = ACK4;
                    scl_next     = 0;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            ACK4: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    state_next   = HOLD;
                    scl_next     = 0;
                    clk_cnt_next = 0;
                    done_next    = 1'b1;
                    ready_next   = 1'b1;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            ///////////////
            SEND_ACK1: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    state_next   = SEND_ACK2;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            SEND_ACK2: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = SEND_ACK3;
                    scl_next     = 1;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            SEND_ACK3: begin
                scl_next = 1;
                if (clk_cnt_reg == 249) begin
                    state_next   = SEND_ACK4;
                    scl_next     = 0;
                    clk_cnt_next = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            SEND_ACK4: begin
                scl_next = 0;
                if (clk_cnt_reg == 249) begin
                    state_next   = HOLD;
                    scl_next     = 0;
                    clk_cnt_next = 0;
                    done_next    = 1'b1;
                    ready_next   = 1'b1;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            ////////////////
            STOP1: begin
                scl_next        = 1'b1;
                tx_data_next[7] = 1'b0;
                if (clk_cnt_reg == 499) begin
                    state_next      = STOP2;
                    clk_cnt_next    = 0;
                    scl_next        = 1'b1;
                    tx_data_next[7] = 1'b1;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            STOP2: begin
                scl_next        = 1'b1;
                tx_data_next[7] = 1'b1;
                if (clk_cnt_reg == 499) begin
                    state_next      = IDLE;
                    clk_cnt_next    = 0;
                    scl_next        = 1'b1;
                    tx_data_next[7] = 1'b1;
                    done_next       = 1'b1;
                    ready_next      = 1'b1;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
        endcase
    end

endmodule
