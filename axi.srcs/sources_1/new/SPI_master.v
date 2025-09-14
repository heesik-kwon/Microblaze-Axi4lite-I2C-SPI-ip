module SPI_Master(
    // global signals
    input        clk,
    input        reset,
    
    input        cpol,
    input        cpha,
    input        start,
    input  [7:0] tx_data,
    output [7:0] rx_data,
    output reg   done,
    output reg   ready,
    // external port
    output SCLK,
    output MOSI,
    input MISO,
    output SS
    );

    localparam IDLE = 0, CP_DELAY = 1, CP0 = 2, CP1 = 3;

    wire r_sclk;
    reg [1:0] state, state_next;
    reg [7:0] temp_tx_data_next, temp_tx_data_reg;
    reg [5:0] sclk_counter_next, sclk_counter_reg;
    reg [2:0] bit_counter_next, bit_counter_reg;
    reg [7:0] temp_rx_data_next, temp_rx_data_reg;

    assign MOSI = temp_tx_data_reg[7];
    assign rx_data = temp_rx_data_reg;
    
    assign r_sclk = ((state_next == CP1) && ~cpha) || ((state_next == CP0) && cpha); 
    assign SCLK = cpol ? ~r_sclk: r_sclk;
    assign SS = (state == IDLE);

    always @(posedge clk, negedge reset) begin
        if (!reset) begin
            state <= IDLE;
            sclk_counter_reg <= 0;
            bit_counter_reg <= 0;
            temp_rx_data_reg <= 0;
            temp_tx_data_reg <= 0;
        end
        else begin
            state <= state_next;
            sclk_counter_reg <= sclk_counter_next;
            bit_counter_reg <= bit_counter_next;
            temp_rx_data_reg <= temp_rx_data_next;
            temp_tx_data_reg <= temp_tx_data_next;
        end
    end

    always @(*) begin
        state_next = state;
        done = 0;
        ready = 0;
        //r_sclk = 0;
        temp_tx_data_next = temp_tx_data_reg;
        temp_rx_data_next = temp_rx_data_reg;
        sclk_counter_next = sclk_counter_reg;
        bit_counter_next = bit_counter_reg;
        case (state)
            IDLE: begin
                temp_tx_data_next = 0;
                done = 0;
                ready = 1;
                if (start) begin
                    state_next = cpha ? CP_DELAY : CP0;
                    temp_tx_data_next = tx_data;
                    ready = 0;
                    sclk_counter_next = 0;
                    bit_counter_next = 0;
                end
            end
            CP_DELAY: begin
                if (sclk_counter_reg == 49) begin
                    sclk_counter_next = 0;
                    state_next = CP0;
                end
                else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end 
            CP0: begin
                //r_sclk = 0;
                if (sclk_counter_reg == 49) begin
                    temp_rx_data_next = {temp_rx_data_reg[6:0], MISO};
                    sclk_counter_next = 0;
                    state_next = CP1;
                end
                else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
            CP1: begin
                //r_sclk = 1;
                if (sclk_counter_reg == 49) begin
                    if (bit_counter_reg == 7) begin
                        done = 1;
                        state_next = IDLE;
                    end
                    else begin
                        temp_tx_data_next = {temp_tx_data_reg[6:0], 1'b0};
                        sclk_counter_next = 0;
                        state_next = CP0;
                        bit_counter_next = bit_counter_reg + 1;
                    end
                end
                else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end 
        endcase
    end
endmodule