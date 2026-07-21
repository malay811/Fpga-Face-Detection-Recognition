/*
To configure the camera module according to our needs of data from it (eg., QVGA), we need SCCB protocol.
We create specific waveform/command here. In response camera will send 2 byte data continuously which contains
RGB. We have 50MHz main source(FPGA) clock here. SCCB protocol clock(SCL) can sustain maximum of 400KHz. 
We are using 100KHz as 'SCL' frequency for that we will use counter to divide 50MHz clock. Camera don't have
it's own clock, so we feed 24MHz 'MCLK' as feed frequency & camera will output data at this freuquency.
FPGA is the master here & camera is slave. In SCCB, the master does not require an ACK from the slave.
The camera does pull SDA low on bit 9 (like I2C), but the FPGA ignores it by tristating SDA (sda_oe=0).
*/

`timescale 1ns / 1ps

module sccb_master #(
    parameter integer CLK_FREQ = 50_000_000
)(
    input wire clk,
    input wire rst,
    ////FPGA is always commanding camera that's why 'SCL' is taken as output.
    output reg sccb_scl,
    inout wire sccb_sda, 
    /*
    In this protocol, the data line (SDA) is shared. For the first 8 bits, the FPGA (Master) talks, and the camera (Slave) listens. On the 9th bit 
   (the ACK bit), they switch roles. The camera talks to say "I received the byte," and the FPGA listens. To say "I received the byte," the camera 
    physically pulls the SDA wire down to 0V. If sccb_sda was a strict output, the FPGA would be forcefully driving 3.3V onto the wire at the exact 
    same moment the camera is forcefully pulling it to 0V. This creates a direct short circuit between the two chips. By making 'sccb_sda' an inout,
    we can use a Tri-state Buffer: 1'bZ.
    Bits 1-8 (sda_oe = 1): The FPGA connects to the wire and drives the data (sda_out).
    Bit 9 (sda_oe = 0): The FPGA outputs 1'bz (High-Impedance). 
    */      
    output reg config_done
    //The camera is actually sending RGB bytes the moment it receives power and the 24 MHz XCLK. 
    //However, until the SCCB master finishes sending all 56 registers, those bytes are completely wrong.
    //wrong resolution, wrong color format, wrong frame rate, after 'config_done' signal it becomes valid.

);
    //At 50 MHz, one clock cycle is 20 ns.
    //To get 100 kHz, we need 500 clock cycles per SCCB bit (50,000,000 / 100,000 = 500).
    localparam integer T_SET = 140; //Keep SCL low, change the SDA data wire and wait for the voltage to settle.
    localparam integer T_HI = 220; //Raise SCL high. The camera reads the SDA wire during this window.
    localparam integer T_LO = 140; //Drop SCL low again before the next bit.
    /*
    The total length of one bit is = T_SET + T_HI + T_LO.
    'T_SET' is setup time = Needs enough time to load data.
    'T_HI' is pulse-width = Needs enough time, so camera can read data.
    'T_LO' is hold time.
    */
    
    /*
    While T_SET, T_HI, and T_LO define the nanosecond level shape of the clock wave. T_PWR, T_RESET, and T_INTERVAL define 
    the millisecond level pauses because it's analog device. The FPGA must care to prevent crashing the camera's internal 
    processor. These constants are loaded into the wcnt (Wait Count) register. When wcnt is loaded with one of these numbers, 
    the State Machine does nothing but counts.   
    */
    localparam integer T_PWR = CLK_FREQ/100; //500,000 cycles. (At 20ns per cycle, this equals 10 ms).
    /*
    When we power our board, the FPGA boots up instantly. The OV7670 camera is an analog/digital hybrid sensor. 
    It takes about 10 milliseconds for its internal voltage regulators to stabilize and its digital end to wake up. 
    If the FPGA starts SCCB commands at it immediately, the camera will ignore them. Loading T_PWR forces the FPGA 
    to wait for 10ms before attempting to send the first register.
    */
    localparam integer T_RESET = CLK_FREQ/200; //5 ms
    /*
    The first register we send to the camera (Register 0x12, value 0x80) is the Software Reset Command. 
    When the camera receives this, it wipes its own memory and reboots. This reboot takes time.
    */
    localparam integer T_INTERVAL = CLK_FREQ/1000; //1 ms
    /*
    For all the other 56 registers, the camera just needs a tiny moment to save the data we sent it before we send the next one. 
    We load 'T_INTERVAL' to pause the FPGA for 1 ms between each register transmission. If we didn't pause here, we would overflow 
    the camera's tiny SCCB receive buffer, and it would drop the configuration data.
    */

    //Read-Only Memory (ROM) Look-Up Table that holds the exact configuration sequence required to boot the OV7670 camera into a usable state.
    localparam integer REG_COUNT = 56; //Total '56' Registers.
    reg [15:0] rom [0:REG_COUNT-1]; //Each are 16bits wide word.
    /*
    Top 8 bits ([15:8]): The target Register Address inside the camera.
    Bottom 8 bits ([7:0]): The Data Value to write into that register.
    */
    initial begin
        rom[0]  = 16'h12_80; //(Reg 0x12 = 0x80): Software Reset. Clears all internal camera state.
        rom[1]  = 16'h11_00; //Clock Divider. Sets internal clock to match external 24MHz exactly (no prescaling).
        rom[2]  = {8'h12,8'h04}; //VGA & RGB Mode. Sets resolution to 640x480 and enables RGB color output through COM7.
        rom[3]  = 16'h0C_00 ; //Disable DCW. Turns off internal downsampling through COM3 (required for full VGA).
        rom[4]  = 16'h3E_00; //Disable Manual Scaling.
        
        //Scaling Parameters. Defines default X and Y scaling curves since downsampling is disabled.
        rom[5]  = 16'h70_3A; 
        rom[6]  = 16'h71_35; 
        rom[7]  = 16'h72_11; 
        rom[8]  = 16'h73_F1; 
        rom[9]  = 16'hA2_02;
         
        rom[10] = 16'h15_00; //Signal Polarity. Leaves PCLK, HREF, and VSYNC at default hardware polarities.
        rom[11] = 16'h40_D0; //RGB565 & Full Range. Forces 565 format and stretches pixel values to the full 0-255 range.
        rom[12] = 16'h8C_00; //Disable RGB444.
        rom[13] = 16'h3A_04; //Sets the exact byte-order sequence the camera outputs on the parallel pins.
        rom[14] = 16'h3D_C0; //Enables Gamma curve correction and auto UV saturation.
        rom[15] = 16'h14_18; //Limits Auto Gain to a 4x multiplier to prevent the image from blowing out in bright light.
        
        //Color Matrix. The camera physically captures a Bayer mosaic (raw pixel colors). These mathematical coefficients 
        //instruct the DSP how to blend the raw pixels into accurate RGB output.        
        rom[16] = 16'h4F_B3; rom[17] = 16'h50_B3; rom[18] = 16'h51_00; rom[19] = 16'h52_3D; rom[20] = 16'h53_A7; rom[21] = 16'h54_E4; rom[22] = 16'h58_9E; 
        
        rom[23] = 16'h3C_78; //HREF Control. Adjusts the horizontal timing signal behavior.
        rom[24] = 16'hB0_84; //Magic Register. An undocumented Omnivision register required for stable color reproduction.
        rom[25] = 16'h6B_0A; //PLL Bypass. Disables internal clock multipliers for stability.
        rom[26] = 16'h01_40; //Blue Gain. Hardcoded baseline for the blue channel.
        rom[27] = 16'h02_60; //Red Gain. Hardcoded baseline for the red channel.
        
        /*
        AWB Matrix. Thec olor-correction algorithms used by the Auto White Balance engine to remove environmental color casts (e.g., removing green tint).
        */
        rom[28] = 16'h43_14;   
        rom[29] = 16'h44_F0;   
        rom[30] = 16'h45_34;   
        rom[31] = 16'h46_58;   
        rom[32] = 16'h47_28;   
        rom[33] = 16'h48_3A;   
        rom[34] = 16'h59_88;   
        rom[35] = 16'h5A_88;   
        rom[36] = 16'h5B_44;   
        rom[37] = 16'h5C_67;   
        rom[38] = 16'h5D_49;   
        rom[39] = 16'h5E_0E;   
        
        //AWB Control. Advanced hardware settings for the white balance algorithm.
        rom[40] = 16'h6C_0A;   
        rom[41] = 16'h6D_55;   
        rom[42] = 16'h6E_11;   
        rom[43] = 16'h6F_9F;  
         
        rom[44] = 16'h6A_40; //Green Gain. Hardcoded baseline for the green channel.
        rom[45] = 16'h55_00; //Brightness. Base level (0).
        rom[46] = 16'h56_40; //Contrast. Base level.
        rom[47] = 16'h13_E7; //Master Enable. This physically turns on the AutoGain (AGC), Auto-White-Balance (AWB) and Auto-Exposure (AEC) processors after their baselines were set above.
        rom[48] = 16'h69_00; //Gain Fix.
        rom[49] = 16'h1E_20; //Orientation. Currently set to horizontal mirroring.
        
        /*
        Hardware Cropping. Adjusts HSTART/HSTOP and VSTART/VSTOP. CMOS sensors have "optical black" pixels at the edges for noise calibration. 
        This crops out those edges so the FPGA only receives the active 640x480 pixels.
        */
        rom[50] = 16'h17_13; //HSTART
        rom[51] = 16'h18_01; //HSTOP
        rom[52] = 16'h32_B6; //HREF  
        rom[53] = 16'h19_02; //VSTART
        rom[54] = 16'h1A_7A; //VSTOP
        rom[55] = 16'h03_0A; //VREF 
    end

    reg sda_out, sda_oe;
    assign sccb_sda = sda_oe ? sda_out : 1'bz; //Tri-State Logic
    //When sda_oe (Output Enable) is 1, the FPGA connects sccb_sda to the wire. When sda_oe is 0, the FPGA physically disconnects from the wire (1'bz), 
    //allowing the camera to drive it without short-circuiting.

    localparam ST_PWR=0, 
               ST_START_A=1, 
               ST_START_B=2, 
               ST_START_C=3,
               ST_BIT_SET=4, 
               ST_BIT_HI=5, 
               ST_BIT_LO=6,
               ST_STOP_A=7, 
               ST_STOP_B=8, 
               ST_INTERVAL=9, 
               ST_DONE=10;

    reg [3:0] state; //Holds the current FSM state (0 to 10).
    reg [19:0] wcnt; //Master delay timer
    reg [1:0] byte_idx; //Counts which byte we are sending: 0=ID, 1=RegAddress, 2=RegData.
    reg [3:0] bit_idx; //Counts the bits within the byte (0 to 8, including the 9th ACK bit).
    reg [5:0] reg_idx; //Counts which of the 56 ROM registers we are currently configuring.
    reg [7:0] cur_byte;
    
    //Combinational logic
    always @* begin
        case (byte_idx)
            2'd0: cur_byte = 8'h42; //Loads the hardcoded Camera ID.
            2'd1: cur_byte = rom[reg_idx][15:8]; //The top 8 bits from the ROM (The Register Address).
            default: cur_byte = rom[reg_idx][7:0]; //The bottom 8 bits from the ROM (The Register Data).
        endcase
    end
    
    //It is Moore Machine. The outputs depend only on the Current State not on the current inputs.
    always @(posedge clk) begin //'if' statement is correct then simulator don't check for 'else if'
        if (rst) begin //Synchronus reset.
            state<=ST_PWR; 
            wcnt<=T_PWR[19:0];
            sccb_scl<=1'b1; 
            sda_out<=1'b1; 
            sda_oe<=1'b1; //Ideal state of SCCB.
            byte_idx<=0; 
            bit_idx<=0; 
            reg_idx<=0; 
            config_done<=1'b0;
        end
        else if (wcnt != 0) begin
            wcnt <= wcnt - 1'b1;
        end
        else begin
            case (state)
            ST_PWR: begin //We prepare to send the first register by moving to ST_START_A.
                state<=ST_START_A; 
                wcnt<=T_HI[19:0];
                sccb_scl<=1'b1; 
                sda_out<=1'b1; 
                sda_oe<=1'b1;
            end
            ST_START_A: begin //Start Condition                    
                state<=ST_START_B; 
                wcnt<=T_HI[19:0];
                sccb_scl<=1'b1; 
                sda_out<=1'b0; 
                sda_oe<=1'b1;
            end
            ST_START_B: begin
                state<=ST_START_C; 
                wcnt<=T_LO[19:0];
                sccb_scl<=1'b0; 
                sda_out<=1'b0; 
                sda_oe<=1'b1;
                byte_idx<=0; 
                bit_idx<=0;
            end
            ST_START_C: begin
                state<=ST_BIT_SET; 
                wcnt<=T_SET[19:0];
                sccb_scl<=1'b0; 
                sda_oe<=1'b1; 
                sda_out<=cur_byte[7];
                //The FPGA grabs the 7th bit (MSB) of 'cur_byte' (which is currently 0x42) and pushes it onto the SDA wire.
            end
            ST_BIT_SET: begin
                state<=ST_BIT_HI; 
                wcnt<=T_HI[19:0];
                sccb_scl<=1'b1;
            end
            ST_BIT_HI: begin
                state<=ST_BIT_LO; 
                wcnt<=T_LO[19:0];
                sccb_scl<=1'b0;
            end
            ST_BIT_LO: begin
                if (bit_idx < 8) begin
                    bit_idx<=bit_idx+1'b1;
                    state<=ST_BIT_SET; 
                    wcnt<=T_SET[19:0];
                    sccb_scl<=1'b0;
                    if (bit_idx+1 < 8) begin
                        sda_oe<=1'b1; 
                        sda_out<=cur_byte[6-bit_idx];
                    end else begin
                        sda_oe<=1'b0;  
                        //When bit_idx hits 7, it means the next clock cycle is the 9th bit. 
                        //The else block triggers, setting sda_oe <= 0. The FPGA physically disconnects from the wire, 
                        //allowing the camera to pull it low to send its ACK.          
                    end
                end
                else begin
                    if (byte_idx < 2) begin //If bit_idx is 8, we drop it here.
                        byte_idx<=byte_idx+1'b1; 
                        bit_idx<=0;
                        state<=ST_BIT_SET; 
                        wcnt<=T_SET[19:0];
                        sccb_scl<=1'b0; 
                        sda_oe<=1'b1;
                        sda_out<=(byte_idx==0) ? rom[reg_idx][15] : rom[reg_idx][7];
                        //(byte_idx==0) checks what the index was. If it was 0, it loads bit 15 (Address MSB). If it was 1, it loads bit 7 (Data MSB).
                    end else begin
                        state<=ST_STOP_A; 
                        wcnt<=T_HI[19:0]; //'T_HI' used here because it gives higher delay.
                        sccb_scl<=1'b0; 
                        sda_oe<=1'b1; 
                        sda_out<=1'b0;
                    end
                end
            end
            ST_STOP_A: begin
                state<=ST_STOP_B; 
                wcnt<=T_HI[19:0];
                sccb_scl<=1'b1; 
                sda_oe<=1'b1; 
                sda_out<=1'b0;
            end
            ST_STOP_B: begin //STOP Condition                      
                state<=ST_INTERVAL;
                sccb_scl<=1'b1; 
                sda_oe<=1'b1; 
                sda_out<=1'b1;
                wcnt<=(reg_idx==0) ? T_RESET[19:0] : T_INTERVAL[19:0];
            end
            ST_INTERVAL: begin
                if (reg_idx == REG_COUNT-1) begin
                    config_done<=1'b1; 
                    state<=ST_DONE; 
                    wcnt<=0;
                end else begin
                    reg_idx<=reg_idx+1'b1;
                    state<=ST_START_A; 
                    wcnt<=T_HI[19:0];
                    sccb_scl<=1'b1; 
                    sda_out<=1'b1; 
                    sda_oe<=1'b1;
                end
            end
            ST_DONE: begin
                state<=ST_DONE; 
                wcnt<=20'd1; //Waits in "else if (wcnt!=0)" branch.
                sccb_scl<=1'b1; 
                sda_out<=1'b1; 
                sda_oe<=1'b1; //Idle state
            end
            default: begin state<=ST_PWR; wcnt<=T_PWR[19:0]; end
            endcase
        end
    end
endmodule