`timescale 1ns / 1ps

module I2C_Slave_hcsr04 (  // ADDR : 1100000
    // global signals
    input        clk,
    input        reset,
    // I2C signals
    input        SCL,
    // input        SDA,
    inout        SDA,
    // export signals
    input         echo,     // HC-SR04 Echo Pulse 
    output        trig
);
    wire        usr;
    wire [11:0] udr;  //ultrasonic distance registe
    wire        done;
    wire [ 6:0] d_state;
    wire        error;

    I2C_Slave_Intf_us U_I2C_Slave_Intf (
        //global signals
        .clk    (clk),
        .reset  (reset),
        // i2C signals
        .SCL    (SCL),
        .SDA    (SDA),
        // export signals
        .led    (),
        .o_state(),
        // UitraSonic port
        .usr    (usr),
        .echo   (echo),     // HC-SR04 Echo Pulse 
        .trig   (trig),
        .udr    (udr),      //ultrasonic distance register
        .done   (done),
        .d_state(d_state),
        .error  (error)
    );

    ultrasonic_sensor U_UltraSonic (
        //global signals
        .clk    (clk),
        .reset  (reset),
        .usr    (usr),      //ultrasonic start register
        .echo   (echo),     // HC-SR04 Echo Pulse 
        .trig   (trig),
        .udr    (udr),      //ultrasonic distance register
        .done   (done),
        .d_state(d_state),
        .error  (error)
    );
endmodule

module I2C_Slave_Intf_us (
    //global signals
    input         clk,
    input         reset,
    // i2C signals
    input         SCL,
    inout         SDA,
    // export signals
    output [ 7:0] led,
    output [ 3:0] o_state,
    // UitraSonic port
    output        usr,      //ultrasonic start register
    output        echo,     // HC-SR04 Echo Pulse 
    input         trig,
    input  [11:0] udr,      //ultrasonic distance register
    input         done,
    input  [ 6:0] d_state,
    input         error
);
    reg [7:0] slv_reg0, slv_next0;
    wire [7:0] slv_reg1, slv_next1;
    wire [7:0] slv_reg2, slv_next2;
    reg [7:0] slv_reg3, slv_next3;

    assign usr = slv_reg0[0];
    assign slv_reg1[7:0] = {1'b0, udr [6:0]};
    assign slv_reg2[7:0] = {1'b0, udr[11:7]};

    /// assign export signals here
    assign led = slv_reg0;

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
            //slv_reg2    <= 0;
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
            // slv_reg2    <= slv_next2;
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
        // slv_next2    = slv_reg2;
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
                        if (rx_data_reg == 8'b1100000_0) begin  // GETDATA
                            state_next = SEND_ACK_DELAY;
                            tx_data_next[7] = 0;
                            clk_cnt_next = 0;
                        end else if (rx_data_reg == 8'b1100000_1) begin  // SENDDATA
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
                            //2'b01: slv_next1 = rx_data_reg;
                            //2'b10: slv_next2 = rx_data_reg;
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

module ultrasonic_sensor (
    input         clk,
    input         reset,
    input         usr,      //ultrasonic start register
    input         echo,     // HC-SR04 Echo Pulse 
    output        trig,
    output [11:0] udr,      //ultrasonic distance register
    output        done,
    output [ 6:0] d_state,
    output        error
);
    wire [11:0] raw_distance;
    wire raw_done;

    wire [2:0] o_state;

    ultrasonic U_ultrasonic (
        .clk     (clk),
        .reset   (reset),
        .start   (usr),           //sw 
        .echo    (echo),          // HC-SR04 Echo Pulse 
        .trig    (trig),          // 10us signal to HC-SR04
        .distance(raw_distance),  // test_distance to fnd
        .done    (raw_done),
        .o_state (o_state),
        .error   ()
    );

    decoder U_decoder (
        .x(o_state),
        .y(d_state)
    );

    median_filter_3samples u_filter (
        .clk           (clk),
        .reset         (reset),
        .new_data_ready(raw_done),
        .data_in       (raw_distance),
        .data_out      (udr)
    );

endmodule


module decoder (
    input [2:0] x,
    output reg [6:0] y
);
    always @(*) begin
        y = 7'b1111111;
        case (x)
            3'd0: y = 7'b0000001;
            3'd1: y = 7'b0000010;
            3'd2: y = 7'b0000100;
            3'd3: y = 7'b0001000;
            3'd4: y = 7'b0010000;
            3'd5: y = 7'b0100000;
            3'd6: y = 7'b1000000;
            default: y = 7'b1111111;
        endcase
    end
endmodule


module median_filter_3samples #(
    parameter DATA_BITS = 12
) (
    input clk,
    input reset,
    input new_data_ready,  //  done 
    input [DATA_BITS-1:0] data_in,
    output reg [DATA_BITS-1:0] data_out
);

    reg [DATA_BITS-1:0] sample_buffer_0, sample_buffer_1, sample_buffer_2;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            sample_buffer_0 <= 0;
            sample_buffer_1 <= 0;
            sample_buffer_2 <= 0;
            data_out <= 0;
        end else if (new_data_ready) begin
            sample_buffer_0 <= sample_buffer_1;
            sample_buffer_1 <= sample_buffer_2;
            sample_buffer_2 <= data_in;

            data_out <= (sample_buffer_0 < sample_buffer_1) ? 
                   ((sample_buffer_1 < sample_buffer_2) ? sample_buffer_1 : 
                   ((sample_buffer_0 < sample_buffer_2) ? sample_buffer_2 : sample_buffer_0)) : 
                   ((sample_buffer_0 < sample_buffer_2) ? sample_buffer_0 : 
                   ((sample_buffer_1 < sample_buffer_2) ? sample_buffer_2 : sample_buffer_1));
        end
    end

endmodule

module ultrasonic #(
    parameter DATA_BITS = 12  //loc2(4000) =11.96
) (
    input clk,
    input reset,
    input start,  //start_btn 
    input echo,  // HC-SR04 Echo Pulse 
    output trig,
    output [DATA_BITS-1:0] distance,  // test_distance to fnd (000.0 cm format)
    output done,
    output [2:0] o_state,
    output error
);

    //parameter
    localparam TRIG_TIME = 10;  //10usec 
    localparam TIMEOUT = 38_000;  // 38msec 
    localparam ECHO_TIMEOUT = 30_000;  // 30msec 
    localparam IDLE_WAITTIME = 60_000;  //60msec
    localparam MIN_VALID_ECHO = 116;  //  under 2cm -> error 
    localparam MAX_VALID_ECHO = 23200;  // over 400cm -> error 

    //state
    localparam IDLE = 3'b000, TRIG = 3'b001, RECEIVE = 3'b010, COUNT = 3'b011,
     RESULT = 3'b100, IDLE_WAIT = 3'b101, ERROR = 3'b110; // ERROR 상태 추가
    reg [2:0] state, next;



    //register
    reg [$clog2(MAX_VALID_ECHO)-1:0]
        e_count, e_count_next;  //echo count (0 ~ 23.2msec) 
    reg [$clog2(IDLE_WAITTIME)-1:0] w_count, w_count_next;
    reg [$clog2(TRIG_TIME)-1:0] t_count, t_count_next;  //trig count

    reg done_reg, done_next;
    reg trig_reg, trig_next;
    reg error_reg, error_next;


    //output
    assign done = done_reg;
    assign o_state = state;
    assign trig = trig_reg;
    assign error = error_reg;


    // assign distance = (error_reg) ? {DATA_BITS{1'b1}} : (e_count * 10) / (58);
    wire [31:0] mult_result;

    assign mult_result = (e_count << 15) + (e_count << 13) + (e_count << 12) + (e_count << 8);
    assign distance = (error_reg) ? {DATA_BITS{1'b1}} : (mult_result >> 18);

    //tick_gen
    wire tick;

    us_tick_gen #(
        .FCOUNT(100)
    ) U_tick_gen (  //1usec
        .clk (clk),
        .rst (reset),
        .tick(tick)
    );

    //state update
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state <= IDLE;
            trig_reg <= 0;
            e_count <= 0;
            w_count <= 0;
            t_count <= 0;
            done_reg <= 0;
            error_reg <= 0;
        end else begin
            state <= next;
            trig_reg <= trig_next;
            e_count <= e_count_next;
            w_count <= w_count_next;
            t_count <= t_count_next;
            done_reg <= done_next;
            error_reg <= error_next;
        end
    end

    //state combinational logic
    always @(*) begin
        next = state;
        trig_next = trig_reg;
        e_count_next = e_count;
        w_count_next = w_count;
        t_count_next = t_count;
        done_next = done_reg;
        error_next = error_reg;

        case (state)
            IDLE: begin
                done_next = 1'b0;
                w_count_next = 0;
                t_count_next = 0;
                error_next = 1'b0;
                if (start) begin
                    next = TRIG;
                end
            end
            TRIG: begin
                trig_next = 1'b1;
                if (t_count == TRIG_TIME - 1) begin
                    next = RECEIVE;
                    t_count_next = 0;
                end else begin
                    if (tick == 1'b1) begin
                        t_count_next = t_count + 1;
                    end
                end
            end
            RECEIVE: begin
                trig_next = 1'b0;
                if (echo) begin
                    e_count_next = 0;
                    next = COUNT;
                end else begin
                    if (w_count == TIMEOUT - 1) begin
                        next = ERROR;
                        error_next = 1'b1;
                        w_count_next = 0;
                    end else begin
                        if (tick == 1'b1) begin
                            w_count_next = w_count + 1;
                        end
                    end
                end
            end
            COUNT: begin
                if (echo == 1'b0) begin
                    if(e_count < MIN_VALID_ECHO || e_count > MAX_VALID_ECHO) begin
                        next = ERROR;
                        error_next = 1'b1;
                    end else begin
                        next = RESULT;
                    end
                end else begin
                    if (e_count >= ECHO_TIMEOUT) begin  // 
                        next = ERROR;
                        error_next = 1'b1;
                    end else if (tick == 1'b1) begin
                        next = COUNT;
                        e_count_next = e_count + 1;
                    end
                end
            end
            RESULT: begin
                if (tick == 1'b1) begin
                    done_next = 1'b1;
                    next = IDLE_WAIT;
                end
            end
            ERROR: begin
                if (tick == 1'b1) begin
                    done_next = 1'b1;
                    next = IDLE_WAIT;
                end
            end
            IDLE_WAIT: begin
                if (w_count == IDLE_WAITTIME - 1) begin
                    next = IDLE;
                    w_count_next = 0;
                end else begin
                    if (tick == 1'b1) begin
                        w_count_next = w_count + 1;
                    end
                end
            end
            default: next = IDLE;
        endcase
    end


endmodule


module us_tick_gen (
    input  clk,
    input  rst,
    output tick
);
    parameter FCOUNT = 100;  //1usec
    reg [$clog2(FCOUNT)-1:0] count_reg, count_next;

    reg tick_reg, tick_next;

    //output
    assign tick = tick_reg;

    //state
    always @(posedge clk, posedge rst) begin
        if (rst == 1'b1) begin
            count_reg <= 0;
            tick_reg  <= 0;
        end else begin
            count_reg <= count_next;
            tick_reg  <= tick_next;
        end
    end

    //next
    always @(*) begin
        count_next = count_reg;
        tick_next  = tick_reg;
        if (count_reg == FCOUNT - 1) begin
            count_next = 1'b0;
            tick_next  = 1'b1;
        end else begin
            count_next = count_reg + 1;
            tick_next  = 1'b0;
        end
    end
endmodule
