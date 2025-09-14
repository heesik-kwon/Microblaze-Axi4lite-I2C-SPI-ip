`timescale 1ns / 1ps

module I2C_Slave_gpio (  // ADDR : 1110000
    // global signals
    input       clk,
    input       reset,
    // I2C signals
    input       SCL,
    // input        SDA,
    inout       SDA,
    // export signals
    inout [7:0] inoutPort
);
    wire [7:0] moder;
    wire [7:0] idr;
    wire [7:0] odr;

    I2C_Slave_Intf_gpio U_I2C_Slave_Intf (
        //global signals
        .clk(clk),
        .reset(reset),
        // i2C signals
        .SCL(SCL),
        .SDA(SDA),
        // export signals
        .led(),
        .o_state(),
        .moder(moder),
        .idr(idr),
        .odr(odr)
    );

    GPIO U_GPIO (
        .moder(moder),
        .idr(idr),
        .odr(odr),
        .inoutPort(inoutPort)
    );
endmodule

module I2C_Slave_Intf_gpio (
    //global signals
    input        clk,
    input        reset,
    // i2C signals
    input        SCL,
    inout        SDA,
    // export signals
    output [7:0] led,
    output [3:0] o_state,
    // internal signals
    output [7:0] moder,
    input  [7:0] idr,
    output [7:0] odr
);
    reg [7:0] slv_reg0, slv_next0;
    wire [7:0] slv_reg1, slv_next1;
    reg [7:0] slv_reg2, slv_next2;
    reg [7:0] slv_reg3, slv_next3;

    /// assign export signals here
    assign moder         = slv_reg0[7:0];
    assign slv_reg1[7:0] = idr;
    assign odr           = slv_reg2[7:0];

    /// end export signals
    localparam  IDLE            = 0, HOLD           = 1, 
                ADDR_LOW        = 2, ADDR_HIGH      = 3,
                REG_LOW         = 4, REG_HIGH       = 5,
                SEND_ACK_DELAY  = 6, SEND_ACK_LOW1  = 7, SEND_ACK_HIGH  = 8, SEND_ACK_LOW2 = 9,
                GETDATA_LOW     = 10, GETDATA_HIGH   = 11,
                SENDDATA_DELAY  = 12, SENDDATA_LOW1 = 13, SENDDATA_HIGH = 14, SENDDATA_LOW2= 15,
                GET_ACK_LOW     = 16, GET_ACK_HIGH  = 17;
    reg [4:0] state, state_next;
    reg [7:0] tx_data_reg, tx_data_next;
    reg [7:0] rx_data_reg, rx_data_next;
    reg [6:0] addr_reg, addr_next;
    reg [2:0] bit_cnt_reg, bit_cnt_next;
    reg [1:0] mode_reg, mode_next;
    reg [$clog2(500)-1:0] clk_cnt_reg, clk_cnt_next;

    reg scl_d, scl_dd;
    reg sda_d, sda_dd;
    always @(posedge clk) begin
        scl_dd <= scl_d;
        scl_d  <= SCL;
    end
    always @(posedge clk) begin
        sda_dd <= sda_d;
        sda_d  <= SDA;
    end
    wire scl_rise = (scl_d == 1 && scl_dd == 0);  // rising edge
    wire scl_fall = (scl_d == 0 && scl_dd == 1);  // falling edge

    wire sda_rise = (sda_d == 1 && sda_dd == 0);  // rising edge
    wire sda_fall = (sda_d == 0 && sda_dd == 1);  // falling edge


    assign SDA = ((state == SEND_ACK_LOW1) | (state == SEND_ACK_HIGH)| (state == SEND_ACK_LOW2)|
                (state == SENDDATA_LOW1) | (state == SENDDATA_HIGH) | (state == SENDDATA_LOW2)) ? tx_data_reg[7] : 1'bz;

    assign o_state = state;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state       <= IDLE;
            tx_data_reg <= 0;
            rx_data_reg <= 0;
            bit_cnt_reg <= 0;
            mode_reg    <= 0;
            clk_cnt_reg <= 0;
            slv_reg0    <= 0;
            //slv_reg1    <= 0;
            slv_reg2    <= 0;
            slv_reg3    <= 0;
            addr_reg    <= 0;
        end else begin
            state       <= state_next;
            tx_data_reg <= tx_data_next;
            rx_data_reg <= rx_data_next;
            bit_cnt_reg <= bit_cnt_next;
            mode_reg    <= mode_next;
            clk_cnt_reg <= clk_cnt_next;
            slv_reg0    <= slv_next0;
            // slv_reg1    <= slv_next1;
            slv_reg2    <= slv_next2;
            slv_reg3    <= slv_next3;
            addr_reg    <= addr_next;
        end
    end

    always @(*) begin
        state_next   = state;
        tx_data_next = tx_data_reg;
        rx_data_next = rx_data_reg;
        bit_cnt_next = bit_cnt_reg;
        mode_next    = mode_reg;
        clk_cnt_next = clk_cnt_reg;
        slv_next0    = slv_reg0;
        // slv_next1    = slv_reg1;
        slv_next2    = slv_reg2;
        slv_next3    = slv_reg3;
        addr_next    = addr_reg;
        case (state)
            IDLE: begin
                bit_cnt_next = 0;
                if (SCL && sda_fall) begin
                    state_next = ADDR_LOW;
                    bit_cnt_next = 0;
                    mode_next = 0;
                end
            end
            HOLD: begin
                if (SCL && sda_rise) begin
                    state_next = IDLE;
                    bit_cnt_next = 0;
                    mode_next = 0;
                end
            end
            ADDR_LOW: begin
                if (scl_rise) begin
                    state_next   = ADDR_HIGH;
                    rx_data_next = {rx_data_reg[6:0], SDA};
                end
            end
            ADDR_HIGH: begin
                if (scl_fall) begin
                    if (bit_cnt_reg == 7) begin
                        bit_cnt_next = 0;
                        mode_next = 0;
                        if (rx_data_reg == 8'b1110000_0) begin  // GETDATA
                            state_next = SEND_ACK_DELAY;
                            tx_data_next[7] = 0;
                            clk_cnt_next = 0;
                        end else if (rx_data_reg == 8'b1110000_1) begin  // SENDDATA
                            state_next = SEND_ACK_DELAY;
                            tx_data_next[7] = 0;
                            clk_cnt_next = 0;
                        end else begin
                            state_next = IDLE;
                        end
                    end else begin
                        state_next   = ADDR_LOW;
                        bit_cnt_next = bit_cnt_reg + 1;
                    end
                end
            end
            REG_LOW: begin
                if (scl_rise) begin
                    state_next = REG_HIGH;
                    addr_next  = {addr_reg[6:0], SDA};
                end
            end
            REG_HIGH: begin
                if (scl_fall) begin
                    if (bit_cnt_reg == 7) begin
                        bit_cnt_next = 0;
                        state_next = SEND_ACK_DELAY;
                        mode_next = 1;
                    end else begin
                        state_next   = REG_LOW;
                        bit_cnt_next = bit_cnt_reg + 1;
                    end
                end
            end
            SEND_ACK_DELAY: begin
                if (clk_cnt_reg == 249) begin
                    clk_cnt_next = 0;
                    state_next = SEND_ACK_LOW1;
                    tx_data_next[7] = 0;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            SEND_ACK_LOW1: begin
                tx_data_next[7] = 0;
                if (scl_rise) begin
                    state_next = SEND_ACK_HIGH;
                    tx_data_next[7] = 0;
                end
            end
            SEND_ACK_HIGH: begin
                tx_data_next[7] = 0;
                if (scl_fall) begin
                    state_next = SEND_ACK_LOW2;
                    tx_data_next[7] = 0;
                end
            end
            SEND_ACK_LOW2: begin
                if (clk_cnt_reg == 249) begin
                    clk_cnt_next = 0;
                    case (mode_reg)
                        2'b00: begin
                            state_next = REG_LOW;
                        end
                        2'b01: begin
                            if (rx_data_reg[0]) begin
                                state_next = SENDDATA_LOW1;
                                case (addr_reg[1:0])
                                    2'b00: tx_data_next = slv_reg0;
                                    2'b01: tx_data_next = slv_reg1;
                                    2'b10: tx_data_next = slv_reg2;
                                    2'b11: tx_data_next = slv_reg3;
                                endcase
                            end else begin
                                state_next = GETDATA_LOW;
                            end
                        end
                        2'b10: begin
                            state_next = HOLD;
                        end
                    endcase
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            GETDATA_LOW: begin
                if (scl_rise) begin
                    state_next   = GETDATA_HIGH;
                    rx_data_next = {rx_data_reg[6:0], SDA};
                end
            end
            GETDATA_HIGH: begin
                if (scl_fall) begin
                    if (bit_cnt_reg == 7) begin
                        bit_cnt_next = 0;
                        state_next   = SEND_ACK_DELAY;
                        case (addr_reg[1:0])
                            2'b00: slv_next0 = rx_data_reg;
                            // 2'b01: slv_next1 = rx_data_reg;
                            2'b10: slv_next2 = rx_data_reg;
                            2'b11: slv_next3 = rx_data_reg;
                        endcase
                        mode_next = 2;
                    end else begin
                        state_next   = GETDATA_LOW;
                        bit_cnt_next = bit_cnt_reg + 1;
                    end
                end
            end
            SENDDATA_DELAY: begin
                if (clk_cnt_reg == 240) begin
                    clk_cnt_next = 0;
                    state_next   = SENDDATA_LOW1;
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            SENDDATA_LOW1: begin
                if (scl_rise) begin
                    state_next = SENDDATA_HIGH;
                end
            end
            SENDDATA_HIGH: begin
                if (scl_fall) begin
                    state_next = SENDDATA_LOW2;
                end
            end
            SENDDATA_LOW2: begin
                if (clk_cnt_reg == 240) begin
                    clk_cnt_next = 0;
                    if (bit_cnt_reg == 7) begin
                        bit_cnt_next = 0;
                        state_next   = GET_ACK_LOW;
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 1;
                        state_next   = SENDDATA_LOW1;
                        tx_data_next = {tx_data_reg[6:0], 1'b0};
                    end
                end else begin
                    clk_cnt_next = clk_cnt_reg + 1;
                end
            end
            GET_ACK_LOW: begin
                if (scl_rise) begin
                    state_next = GET_ACK_HIGH;
                end
            end
            GET_ACK_HIGH: begin
                if (scl_fall) begin
                    if (SDA) begin
                        addr_next = addr_reg + 1;
                        case (addr_next[1:0])
                            2'b00: tx_data_next = slv_reg0;
                            2'b01: tx_data_next = slv_reg1;
                            2'b10: tx_data_next = slv_reg2;
                            2'b11: tx_data_next = slv_reg3;
                        endcase
                        state_next = SENDDATA_LOW1;
                    end else begin
                        state_next = HOLD;
                    end
                end
            end
        endcase
    end
endmodule

module GPIO (
    input  [7:0] moder,
    output [7:0] idr,
    input  [7:0] odr,
    inout  [7:0] inoutPort
);

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin
            assign inoutPort[i] = moder[i] ? odr[i] : 1'bz;  //output mode
            assign idr[i] = ~moder[i] ? inoutPort[i] : 1'bz;  //input mode
        end
    endgenerate
endmodule
