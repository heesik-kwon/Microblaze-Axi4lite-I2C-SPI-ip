`timescale 1ns / 1ps
module SPI_Slave (
    input        clk,
    input        reset,
    input        SCLK,
    input        MOSI,
    output       MISO,
    input        SS,
    output [3:0] fnd_comm,
    output [7:0] fnd_font
);

    wire [ 7:0] si_data;
    wire        si_done;
    wire [ 7:0] so_data;
    wire        so_start;
    wire        so_done;

    wire [ 1:0] counter;
    wire [ 3:0] y;
    wire [15:0] num;
    wire [3:0] x0, x1, x2, x3;

    SPI_Slave_Intf U_SPI_Slave_Intf (
        .clk(clk),
        .reset(reset),
        .SCLK(SCLK),
        .MOSI(MOSI),
        .MISO(MISO),
        .SS(SS),
        .si_data(si_data),
        .si_done(si_done),
        .so_data(so_data),
        .so_start(so_start),
        .so_done(so_done)
    );

    SPI_Slave_Reg U_SPI_Slave_Reg (
        .clk(clk),
        .reset(reset),
        .ss_n(SS),
        .si_data(si_data),
        .si_done(si_done),
        .so_data(so_data),
        .so_start(so_start),
        .so_done(so_done),
        .fndData(num)
    );

    clk_div U_clk_div (
        .clk(clk),
        .rst(reset),
        .clk_div(clk_div)
    );

    counter_4 U_counter (
        .clk(clk_div),
        .rst(reset),
        .counter(counter)
    );

    digit_split U_split (
        .num(num),
        .x0 (x0),
        .x1 (x1),
        .x2 (x2),
        .x3 (x3)
    );

    mux_4 U_Mux (
        .x0(x0),
        .x1(x1),
        .x2(x2),
        .x3(x3),
        .counter(counter),
        .y(y)
    );

    dec2x4 U_dec (
        .counter(counter),
        .y(fnd_comm)
    );

    bcdtoseg U_BCD (
        .bcd(y),
        .seg(fnd_font)
    );
endmodule

module SPI_Slave_Intf (
    input        clk,
    input        reset,
    input        SCLK,
    input        MOSI,
    output       MISO,
    input        SS,
    output [7:0] si_data,
    output       si_done,
    input  [7:0] so_data,
    input        so_start,
    output       so_done
);

    reg sclk_sync0, sclk_sync1;

    reg ss_d, ss_dd;
    wire ss_fall_edge, ss_rise_edge;
    assign ss_fall_edge = (~ss_d) & ss_dd;
    assign ss_rise_edge = (ss_d) & (~ss_dd);

    always @(posedge clk) begin
        ss_d  <= SS;
        ss_dd <= ss_d;
    end

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            sclk_sync0 <= 0;
            sclk_sync1 <= 0;
        end else begin
            sclk_sync0 <= SCLK;
            sclk_sync1 <= sclk_sync0;
        end
    end

    wire sclk_rising = sclk_sync0 & ~sclk_sync1;
    wire sclk_falling = ~sclk_sync0 & sclk_sync1;

    // Slave Input Circuit(MOSI)
    localparam SI_IDLE = 0, SI_PHASE = 1;

    reg si_state, si_state_next;
    reg [7:0] si_data_reg, si_data_next;
    reg [2:0] si_bit_cnt_reg, si_bit_cnt_next;
    reg si_done_reg, si_done_next;

    assign si_done = si_done_reg;
    assign si_data = si_data_reg;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            si_state <= SI_IDLE;
            si_data_reg <= 0;
            si_bit_cnt_reg <= 0;
            si_done_reg <= 0;
        end else begin
            si_state       <= si_state_next;
            si_data_reg    <= si_data_next;
            si_bit_cnt_reg <= si_bit_cnt_next;
            si_done_reg    <= si_done_next;
        end
    end

    always @(*) begin
        si_state_next = si_state;
        si_data_next = si_data_reg;
        si_bit_cnt_next = si_bit_cnt_reg;
        si_done_next = 0;
        case (si_state)
            SI_IDLE: begin
                si_done_next = 1'b0;
                if (ss_fall_edge) begin
                    si_state_next   = SI_PHASE;
                    si_bit_cnt_next = 0;
                end
            end
            SI_PHASE: begin
                if (ss_rise_edge) begin
                    si_state_next = SI_IDLE;
                end else begin
                    if (sclk_rising) begin  // sclk_rising
                        si_data_next = {si_data_reg[6:0], MOSI};
                        if (si_bit_cnt_reg == 7) begin
                            si_done_next = 1'b1;
                            si_bit_cnt_next = 0;
                            si_state_next = SI_IDLE;  //**
                        end else begin
                            si_bit_cnt_next = si_bit_cnt_reg + 1;
                        end
                    end
                end

            end
        endcase
    end

    // Slave Output Circuit(MISO)
    localparam SO_IDLE = 0, SO_PHASE = 1;

    reg so_state, so_state_next;
    reg [7:0] so_data_reg, so_data_next;
    reg [2:0] so_bit_cnt_reg, so_bit_cnt_next;
    reg so_done_reg, so_done_next;

    assign so_done = so_done_reg;
    assign MISO = (~SS & so_state == SO_PHASE) ? so_data_reg[7] : 1'bz;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            so_state       <= SO_IDLE;
            so_data_reg    <= 1'b0;
            so_bit_cnt_reg <= 0;
            so_done_reg    <= 0;
        end else begin
            so_state       <= so_state_next;
            so_data_reg    <= so_data_next;
            so_bit_cnt_reg <= so_bit_cnt_next;
            so_done_reg    <= so_done_next;
        end
    end

    always @(*) begin
        so_state_next = so_state;
        so_bit_cnt_next = so_bit_cnt_reg;
        so_data_next = so_data_reg;
        so_done_next = so_done_reg;  // 수정 ********** so_done_reg;
        case (so_state)
            SO_IDLE: begin
                so_done_next = 1'b0;
                if (!SS && so_start) begin
                    so_state_next = SO_PHASE;
                    so_bit_cnt_next = 0;
                    so_data_next = so_data;
                end
            end
            SO_PHASE: begin
                if (ss_fall_edge) begin
                    so_state_next = SO_IDLE;
                end else begin
                    if (sclk_falling) begin
                        so_data_next = {so_data_reg[6:0], 1'b0};
                        if (so_bit_cnt_reg == 7) begin
                            so_bit_cnt_next = 0;
                            so_done_next = 1'b1;
                            so_state_next = SO_IDLE;
                        end else begin
                            so_bit_cnt_next = so_bit_cnt_reg + 1;
                        end
                    end
                end
            end
        endcase
    end
endmodule

module SPI_Slave_Reg (
    input             clk,
    input             reset,
    input             ss_n,
    input      [ 7:0] si_data,
    input             si_done,
    // input          so_ready,
    output reg [ 7:0] so_data,
    output            so_start,
    output     [15:0] fndData,
    input             so_done
);
    localparam IDLE = 0, ADDR_PHASE = 1, WRITE_PHASE = 2, 
    READ_DEALY =3, READ_PHASE = 4, WRITE_PHASE_BLINK= 5, READ_PHASE_BLINK =6;
    reg [7:0] slv_reg_reg [0:3];
    reg [7:0] slv_reg_next[0:3];
    reg [2:0] state, state_next;
    reg [1:0] addr_reg, addr_next;
    reg so_start_reg, so_start_next;
    reg [$clog2(50)-1:0] clk_counter_reg, clk_counter_next;

    assign so_start = so_start_reg;
    assign fndData  = {slv_reg_reg[1], slv_reg_reg[0]};

    reg ss_d, ss_dd;
    wire ss_fall_edge, ss_rise_edge;
    assign ss_fall_edge = (~ss_d) & (ss_dd);
    assign ss_rise_edge = (ss_d) & (~ss_dd);

    always @(posedge clk) begin
        ss_d  <= ss_n;
        ss_dd <= ss_d;
    end

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state           <= IDLE;
            addr_reg        <= 2'b0;
            so_start_reg    <= 1'b0;
            clk_counter_reg <= 0;
            slv_reg_reg[0]  <= 0;
            slv_reg_reg[1]  <= 0;
            slv_reg_reg[2]  <= 0;
            slv_reg_reg[3]  <= 0;
        end else begin
            state           <= state_next;
            addr_reg        <= addr_next;
            so_start_reg    <= so_start_next;
            clk_counter_reg <= clk_counter_next;
            slv_reg_reg[0]  <= slv_reg_next[0];
            slv_reg_reg[1]  <= slv_reg_next[1];
            slv_reg_reg[2]  <= slv_reg_next[2];
            slv_reg_reg[3]  <= slv_reg_next[3];
        end
    end

    always @(*) begin
        state_next       = state;
        addr_next        = addr_reg;
        so_start_next    = so_start_reg;
        so_data          = 0;
        clk_counter_next = clk_counter_reg;
        slv_reg_next[0]  = slv_reg_reg[0];
        slv_reg_next[1]  = slv_reg_reg[1];
        slv_reg_next[2]  = slv_reg_reg[2];
        slv_reg_next[3]  = slv_reg_reg[3];
        case (state)
            IDLE: begin
                so_start_next = 1'b0;
                if (ss_fall_edge) begin
                    state_next = ADDR_PHASE;
                end
            end
            ADDR_PHASE: begin
                so_start_next = 1'b0;
                if (ss_rise_edge) begin
                    if (si_data[7]) begin
                        state_next = WRITE_PHASE_BLINK;
                    end else begin
                        state_next = READ_PHASE_BLINK;
                    end
                end else begin
                    if (si_done) begin
                        addr_next = si_data[1:0];
                    end
                end
            end
            WRITE_PHASE: begin
                so_start_next = 1'b0;
                if (ss_rise_edge) begin
                    state_next = IDLE;
                end else begin
                    if (si_done) begin
                        //
                        case (addr_reg)
                            2'b00: slv_reg_next[0] = si_data;
                            2'b01: slv_reg_next[1] = si_data;
                            2'b10: slv_reg_next[2] = si_data;
                            2'b11: slv_reg_next[3] = si_data;
                        endcase
                    end
                end
            end
            READ_PHASE: begin
                if (ss_rise_edge) begin
                    state_next = IDLE;
                end else begin
                    so_start_next = 1'b1;
                    so_data = slv_reg_next[addr_reg];
                end
            end
            WRITE_PHASE_BLINK: begin
                so_start_next = 1'b0;
                if (ss_fall_edge) begin
                    state_next = WRITE_PHASE;
                end
            end
            READ_PHASE_BLINK: begin
                so_start_next = 1'b0;
                if (ss_fall_edge) begin
                    state_next = READ_PHASE;
                end
            end
        endcase
    end
endmodule


module FND (
    input clk,
    input rst,
    input CS,
    input done,
    input [7:0] data,
    output reg [15:0] num
);

    localparam IDLE = 0, L_BYTE = 1, H_BYTE = 2;
    reg [1:0] state, next;

    reg [7:0] l_byte_reg, h_byte_reg;
    reg [7:0] l_byte_next, h_byte_next;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            l_byte_reg <= 0;
            h_byte_reg <= 0;
            num <= 0;
        end else begin
            state <= next;
            l_byte_reg <= l_byte_next;
            h_byte_reg <= h_byte_next;

            // 완료 시점에서 num 업데이트
            if (state == H_BYTE && next == IDLE) begin
                num <= {8'b00, h_byte_next};
            end
        end
    end

    always @(*) begin
        next = state;
        l_byte_next = l_byte_reg;
        h_byte_next = h_byte_reg;

        case (state)
            IDLE: begin
                if (!CS) next = L_BYTE;
            end

            L_BYTE: begin
                if (!CS && done) begin
                    l_byte_next = data;
                    next = H_BYTE;
                end
            end

            H_BYTE: begin
                if (!CS && done) begin
                    h_byte_next = data;
                    next = IDLE;
                end
            end
        endcase
    end

endmodule

module clk_div (
    input clk,
    input rst,
    output reg clk_div
);

    reg [$clog2(100_000)-1:0] count;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            clk_div <= 0;
            count   <= 0;
        end else begin
            if (count == 100_000 - 1) begin
                clk_div <= 1;
                count   <= 0;
            end else begin
                clk_div <= 0;
                count   <= count + 1;
            end
        end
    end
endmodule

module counter_4 (
    input clk,
    input rst,
    output reg [1:0] counter
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 0;
        end else begin
            counter <= counter + 1;
        end
    end
endmodule

module mux_4 (
    input [3:0] x0,
    input [3:0] x1,
    input [3:0] x2,
    input [3:0] x3,
    input [1:0] counter,
    output reg [3:0] y
);
    always @(*) begin
        case (counter)
            0: y = x0;
            1: y = x1;
            2: y = x2;
            3: y = x3;
            default: y = 0;
        endcase
    end
endmodule

module dec2x4 (
    input [1:0] counter,
    output reg [3:0] y
);

    always @(*) begin
        case (counter)
            0: y = 4'b1110;
            1: y = 4'b1101;
            2: y = 4'b1011;
            3: y = 4'b0111;
            default: y = 4'b0000;
        endcase
    end

endmodule

module digit_split (
    input [15:0] num,
    output reg [3:0] x0,
    output reg [3:0] x1,
    output reg [3:0] x2,
    output reg [3:0] x3
);

    always @(*) begin
        x0 = num % 10;
        x1 = num / 10 % 10;
        x2 = num / 100 % 10;
        x3 = num / 1000 % 10;
    end

endmodule

module bcdtoseg (
    input [3:0] bcd,
    output reg [7:0] seg
);

    always @(*) begin
        case (bcd)
            4'h0: seg = 8'hC0;
            4'h1: seg = 8'hF9;
            4'h2: seg = 8'hA4;
            4'h3: seg = 8'hB0;
            4'h4: seg = 8'h99;
            4'h5: seg = 8'h92;
            4'h6: seg = 8'h82;
            4'h7: seg = 8'hF8;
            4'h8: seg = 8'h80;
            4'h9: seg = 8'h90;
            4'hA: seg = 8'h88;
            4'hB: seg = 8'h83;
            4'hC: seg = 8'hC6;
            4'hD: seg = 8'hA1;
            4'hE: seg = 8'h7F;  // dot
            4'hF: seg = 8'hff;
            default: seg = 8'hff;
        endcase
    end
endmodule
