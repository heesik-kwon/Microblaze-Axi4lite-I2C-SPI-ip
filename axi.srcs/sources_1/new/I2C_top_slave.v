`timescale 1ns / 1ps

module I2C_Top_Slave (
    // global signals
    input        clk,
    input        reset,
    // I2C signals
    input        SCL,
    inout        SDA,
    
    // GPIO signals
    inout  [7:0] inoutPort,
    // UltraSonic signals
    input        echo,       // HC-SR04 Echo Pulse 
    output       trig,
    
    // SPI signals
    input        SCLK,
    input        MOSI,
    output       MISO,
    input        SS,
    // FND signals
    output [3:0] fnd_comm,
    output [7:0] fnd_font
);

    SPI_Slave U_SPI_FND (
        .clk(clk),
        .reset(reset),
        .SCLK(SCLK),
        .MOSI(MOSI),
        .MISO(MISO),
        .SS(SS),
        .fnd_comm(fnd_comm),
        .fnd_font(fnd_font)
    );

    I2C_Slave_gpio U_GPIO (  // ADDR : 1110000
        .clk(clk),
        .reset(reset),
        .SCL(SCL),
        .SDA(SDA),
        .inoutPort(inoutPort)
    );

    I2C_Slave_hcsr04 U_UltraSonic (  // ADDR : 1100000
        .clk  (clk),
        .reset(reset),
        .SCL  (SCL),
        .SDA  (SDA),
        .echo (echo),
        .trig (trig)
    );
endmodule
