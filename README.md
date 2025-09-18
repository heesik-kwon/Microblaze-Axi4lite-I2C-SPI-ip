# 💻 AXI4-Lite 기반 SPI/I2C 통합 System
> SPI와 I2C Master/Slave를 설계하고, UVM 환경에서 동작을 검증하였습니다.
> 또한 Microblaze CPU와 AXI_I2C Bridge, AXI_API Bridge를 연동하여 실제 보드 제어까지 수행함으로써, 설계 단계에서 구현한 기능이 하드웨어 환경에서도 정상적으로 동작함을 확인하였습니다.

---


# 📌 프로젝트 개요

| 항목             | 내용                                                   |
|------------------|--------------------------------------------------------|
| **⏱️ 개발 기간** | 2024.08.24 ~ 2024.09.01                               |
| **🖥️ 개발 환경**  | Vivado, VCS, VSCode                               |
| **💻 언어**       | System Verilog, Verilog                                           |

---

# 🧰 Block Diagram
<img width="1583" height="707" alt="image" src="https://github.com/user-attachments/assets/8e1ed11a-676c-4e06-9b5c-8edce6dded6d" />

---

# 📉 SPI

## 1️⃣ Logic Analyzer(Write / Read)
<img width="2383" height="473" alt="image" src="https://github.com/user-attachments/assets/78323098-dca5-4a45-abb9-e51a02410e93" />

## 2️⃣ RTL Simulation
<img width="1506" height="829" alt="image" src="https://github.com/user-attachments/assets/f1fdda0d-38a1-489e-8a59-98839af87cb8" />


---

# 📈 I2C

## 1️⃣ Logic Analyzer(Write / Read)
<img width="2240" height="758" alt="image" src="https://github.com/user-attachments/assets/dadc48ef-9603-4b9d-b9c8-e3a2aa1e876b" />

## 2️⃣ RTL Simulation
<img width="1658" height="836" alt="image" src="https://github.com/user-attachments/assets/a83f102c-24fe-46d8-9a2b-4c0b8aa902af" />


---

# 🛠️ UVM
## 1️⃣ Source Code

### Sequence
```verilog
class i2c_seq_item extends uvm_sequence_item;

    rand bit reset;
    rand bit [7:0] tx_data;
         logic [7:0] rx_data;
         logic       done;
         logic [1:0] mode;
    rand bit       enable;
         logic [7:0] inoutPort1;
         logic [7:0] inoutPort2;

    // Reference reg 변경
    rand bit [6:0] SLAVE_ADDR;
    rand bit [1:0] REG_NUM;

    // 제약 조건
    constraint valid_addr_c {
        SLAVE_ADDR inside {7'h60, 7'h70}; // 1100_000 or 1110_000
    }

    constraint reg_num_c {
        REG_NUM inside {0, 1, 2};
    }

endclass
```

### Driver
```verilog
if (i2c_item.REG_NUM == 1) begin
    i_if.tx_data = {i2c_item.SLAVE_ADDR, 1'b1}; // LSB = read for IDR
end else begin
    i_if.tx_data = {i2c_item.SLAVE_ADDR, 1'b0}; // LSB = write for MOR or ODR
end
```
```verilog
if (i2c_item.REG_NUM == 1) begin
    i_if.mode = 3; // LSB = read for IDR
end else begin
    i_if.mode = 0; // LSB = write for MOR or ODR
end
```

### Monitor
```verilog
virtual task run_phase(uvm_phase phase);
    forever begin
        @(posedge i_if.clk);
        @(posedge i_if.done);
        @(posedge i_if.done);
        @(posedge i_if.done);
        @(posedge i_if.done);
        @(posedge i_if.done);
        @(posedge i_if.clk);

        i2c_item.rx_data   = i_if.rx_data;
        i2c_item.tx_data   = i_if.tx_data;
        i2c_item.inoutPort1 = i_if.inoutPort1;
        i2c_item.inoutPort2 = i_if.inoutPort2;
        i2c_item.done      = i_if.done;
        i2c_item.mode      = i_if.mode;
        i2c_item.enable    = i_if.enable;

        @(posedge i_if.clk);

        i2c_item.REG_NUM   = i_if.REG_NUM;
        i2c_item.SLAVE_ADDR = i_if.SLAVE_ADDR;

        `uvm_info("MON", $sformatf("SLAVE_ADDR: %0b, REG_NUM: %0d, tx_data: %0b, inoutPort1: %0b, inoutPort2: %0b",
                    i2c_item.SLAVE_ADDR, i2c_item.REG_NUM, i2c_item.tx_data,
                    i2c_item.inoutPort1, i2c_item.inoutPort2), UVM_NONE);

        send.write(i2c_item);
    end
endtask
```
### Scoreboard
```verilog
// 0: MODER
2'b00: begin // MODER 쓰기
    if (i2c_item.SLAVE_ADDR == 7'b1100000) begin
        moder_reg1 = i2c_item.tx_data;
        $display("MODER1 updated to: %0b", moder_reg1);
    end else if (i2c_item.SLAVE_ADDR == 7'b1110000) begin
        moder_reg2 = i2c_item.tx_data;
        $display("MODER2 updated to: %0b", moder_reg2);
    end
end
```

```verilog
// 1: IDR
2'b01: begin
    expected = 8'hZZ; // 초기값
    if (i2c_item.SLAVE_ADDR == 7'b1100000) begin
        for (int i = 0; i < 8; i++) begin
            if (moder_reg1[i] == 1'b0) begin
                if (i2c_item.inoutPort1[i] !== i2c_item.rx_data[i]) begin
                    `uvm_error("SCO", $sformatf("IDR1[%0d] mismatch: expected %0b, got %0b",
                                i, i2c_item.rx_data[i], i2c_item.inoutPort1[i]));
                end
            end
        end
        $display("IDR1 PASS: %0b", i2c_item.rx_data);
    end else if (i2c_item.SLAVE_ADDR == 7'b1110000) begin
        for (int i = 0; i < 8; i++) begin
            if (moder_reg2[i] == 1'b0) begin
                if (i2c_item.inoutPort2[i] !== i2c_item.rx_data[i]) begin
                    `uvm_error("SCO", $sformatf("IDR2[%0d] mismatch: expected %0b, got %0b",
                                i, i2c_item.rx_data[i], i2c_item.inoutPort2[i]));
                end
            end
        end
        $display("IDR2 PASS: %0b", i2c_item.rx_data);
    end
end
```

```verilog
// 2: ODR
2'b10: begin
    expected = 8'hZZ;
    if (i2c_item.SLAVE_ADDR == 7'b1100000) begin
        odr_reg1 = i2c_item.tx_data; // write 시 ODR 갱신
        for (int i = 0; i < 8; i++) begin
            if (moder_reg1[i] == 1'b1 && i2c_item.inoutPort1[i] !== odr_reg1[i]) begin
                `uvm_error("SCO", $sformatf("ODR1[%0d] mismatch: expected %0b, got %0b",
                            i, odr_reg1[i], i2c_item.inoutPort1[i]));
            end
        end
        $display("ODR1 PASS: %0b", odr_reg1);
    end else if (i2c_item.SLAVE_ADDR == 7'b1110000) begin
        odr_reg2 = i2c_item.tx_data; // write 시 ODR 갱신
        for (int i = 0; i < 8; i++) begin
            if (moder_reg2[i] == 1'b1 && i2c_item.inoutPort2[i] !== odr_reg2[i]) begin
                `uvm_error("SCO", $sformatf("ODR2[%0d] mismatch: expected %0b, got %0b",
                            i, odr_reg2[i], i2c_item.inoutPort2[i]));
            end
        end
        $display("ODR2 PASS: %0b", odr_reg2);
    end
end
```

## 2️⃣ UVM Report
<img width="1672" height="916" alt="image" src="https://github.com/user-attachments/assets/4437348d-77a4-45e3-ab53-cf07f9a11c6f" />

시뮬레이션이 정상적으로 종료되었고 오류 없이 PASS된 것을 확인할 수 있습니다.

<img width="381" height="358" alt="image" src="https://github.com/user-attachments/assets/128ec8b3-d624-4996-9c0b-7399a135e6a1" />

---

# 📖 UVM Verification Result
<img width="1803" height="660" alt="image" src="https://github.com/user-attachments/assets/48cb2f39-50ac-45e7-ae50-09c576958d52" />

