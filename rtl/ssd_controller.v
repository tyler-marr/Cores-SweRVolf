


module ssd_controller #(
    parameter [3:0] NUM_SEGMENTS = 4,
    parameter [31:0] HOLD_FREQ = 2000,
    parameter [31:0] clk_freq_hz = 0
    ) (
    //typical clk, rst signals
    input wire          i_clk,
    input wire          i_rst,

    //wishbone bus signals
    input wire [5:0]    i_wb_adr,
    input wire [31:0]   i_wb_dat,
    input wire [3:0]    i_wb_sel,
    input wire          i_wb_we,
    input wire          i_wb_cyc,
    input wire          i_wb_stb,
    output reg [31:0]   o_wb_rdt,
    output reg          o_wb_ack,
    
    //SSD specfic signals 
    output reg [NUM_SEGMENTS-1:0]   o_anode,    //active low SSD selector
    output reg [7:0]                o_cathode   //active high Segment enable
    );

    localparam HOLD_TIMER_MAX = clk_freq_hz/HOLD_FREQ;

    // SSD value storage
    //NUM_SEGMENTS lots of 8bits for SSD data
    reg [7:0] ssd_values[NUM_SEGMENTS-1:0]; 
    reg [7:0] ssd_hex_values[15:0];

    // SSD internal data   
    reg  [31:0] hold_timer;         //timer to slow down SSD flashing
    reg  [1:0] index;               // [($clog2(NUM_SEGMENTS))-1:0]
    reg        ssd_en_hexdecode;    //enables/diables hex decode of SSD values 

    //wishbone logic for write_enable
    wire [0:0] reg_we = i_wb_cyc & i_wb_stb & i_wb_we & !o_wb_ack; 

    wire [7:0] ssd_current_raw_value = ssd_values[index];
    wire [7:0] ssd_current_hex_value = ssd_hex_values[ssd_current_raw_value];

    initial begin 
        index               = 0;
        hold_timer          = 0;
        o_cathode           = 8'h3;
        o_anode             = ~(4'h1);
        ssd_en_hexdecode    = 0;

        ssd_values[0] = 0;
        ssd_values[1] = 1;
        ssd_values[2] = 2;
        ssd_values[3] = 3;

        ssd_hex_values[4'h0] = 8'b11000000; //0
        ssd_hex_values[4'h1] = 8'b11111001; //1
        ssd_hex_values[4'h2] = 8'b10100100; //2
        ssd_hex_values[4'h3] = 8'b10110000; //3
        ssd_hex_values[4'h4] = 8'b10011001; //4
        ssd_hex_values[4'h5] = 8'b10010010; //5
        ssd_hex_values[4'h6] = 8'b10000010; //6
        ssd_hex_values[4'h7] = 8'b11111000; //7
        ssd_hex_values[4'h8] = 8'b10000000; //8
        ssd_hex_values[4'h9] = 8'b10010000; //9
        ssd_hex_values[4'hA] = 8'b10001000; //A
        ssd_hex_values[4'hB] = 8'b10000011; //B
        ssd_hex_values[4'hC] = 8'b11000110; //C
        ssd_hex_values[4'hD] = 8'b10100001; //D
        ssd_hex_values[4'hE] = 8'b10000110; //E
        ssd_hex_values[4'hF] = 8'b10001110; //F
        
    end 

    always @(posedge i_clk, posedge i_rst) begin

        if (i_rst) begin 
            index               <= 0;
            hold_timer          <= 0;
            o_cathode           <= 8'h3;
            o_anode             <= ~(4'h1);
            ssd_en_hexdecode    <= 0;

            ssd_values[0] <= 0;
            ssd_values[1] <= 0;
            ssd_values[2] <= 0;
            ssd_values[3] <= 0;
        end else begin

            //time period to hold a single SSD lit before movin to the next
            if (hold_timer == HOLD_TIMER_MAX) begin
                hold_timer <= 0;

                //Num SSDs is a pow of 2 so natual overflow is fine
                index <= index + 1; 
            end else begin 
                hold_timer <= hold_timer + 1;
            end 

            //either hex decode to the SSD or use raw values
            o_cathode <= ssd_en_hexdecode ? ssd_current_hex_value : ssd_current_raw_value;
            //anode is active low so invert the selection register
            o_anode <= ~(8'h1 << index);

            //wishbone logic for acknowledge 
            o_wb_ack <= i_wb_cyc & !o_wb_ack;

            if (reg_we) begin
                //the lower 2 bits are truncated as wishbone addressing is
                //chunked into 32 bit blocks, to achieve byte addressing the
                //selection flags (i_wb_sel) identifys the valid bytes in
                //i_wb_data
                case (i_wb_adr[5:2])
                    0: begin
                        if (i_wb_sel[0]) ssd_en_hexdecode <= |i_wb_dat[7:0];
                    end
                    1: begin
                        if (i_wb_sel[0]) ssd_values[0] <= i_wb_dat[7:0];
                        if (i_wb_sel[1]) ssd_values[1] <= i_wb_dat[15:8];
                        if (i_wb_sel[2]) ssd_values[2] <= i_wb_dat[23:16];
                        if (i_wb_sel[3]) ssd_values[3] <= i_wb_dat[31:24];
                    end
                endcase
                /*  register map look like
                    struct SSD_con {
                        uint8_t ssd_en_hexdecode;
                        uint8_t unused[3];
                        uint8_t ssd_values[4];
                    }
                */
            
            end
        end
    end
endmodule