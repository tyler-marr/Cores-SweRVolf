


module ssd_controller
    #(parameter [3:0] NUM_SEGMENTS = 8) //paramaters
   (
    input wire          i_clk,
    input wire          i_rst,

    input wire [5:0]    i_wb_adr,
    input wire [31:0]   i_wb_dat,
    input wire [3:0]    i_wb_sel,
    input wire          i_wb_we,
    input wire          i_wb_cyc,
    input wire          i_wb_stb,
    output reg [31:0]   o_wb_rdt,
    output reg          o_wb_ack,
    
    output reg [NUM_SEGMENTS-1:0]    o_anode,
    output wire [NUM_SEGMENTS-1:0]    o_cathode);

    // SSD value storage
    reg [NUM_SEGMENTS-1:0] ssd_values[7:0]; //NUM_SEGMENTS lots of 8bits for SSD data

    //SSD internal data   
    reg  [9:0] scaler;              //prescaler to slow down SSD flashing
    reg  [7:0] SSDhex;              //hex translated value for SSD to display 
    reg        ssd_en_hexdecode;
    wire [2:0] index;        //active SSD
    // wire [$clog2(NUM_SEGMENTS)-1:0] index;        //active SSD
    wire [7:0] ssd_current_value;   //'live' value of active SSD

    wire reg_we = i_wb_cyc & i_wb_stb & i_wb_we & !o_wb_ack;

    //either hex decode to the SSD or use raw values 
    assign o_cathode = ssd_en_hexdecode ? SSDhex : ~ssd_current_value[7:0];
    assign ssd_current_value = ssd_values[index];
    assign index = scaler[9:7];

    always @(posedge i_clk) begin
        o_wb_ack <= i_wb_cyc & !o_wb_ack;
        scaler <= scaler + 1;

        if (reg_we)

            case (i_wb_adr[5:2])
                //map reg writes to the approiate register based on address
                0: begin
                    if (i_wb_sel[0]) ssd_values[0] <= i_wb_dat[7:0];
                    if (i_wb_sel[1]) ssd_values[1] <= i_wb_dat[15:8];
                    if (i_wb_sel[2]) ssd_values[2] <= i_wb_dat[23:16];
                    if (i_wb_sel[3]) ssd_values[3] <= i_wb_dat[31:24];

                end
                1: begin
                    if (i_wb_sel[0]) ssd_values[4] <= i_wb_dat[7:0];
                    if (i_wb_sel[1]) ssd_values[5] <= i_wb_dat[15:8];
                    if (i_wb_sel[2]) ssd_values[6] <= i_wb_dat[23:16];
                    if (i_wb_sel[3]) ssd_values[7] <= i_wb_dat[31:24];
                end
                3: begin
                   ssd_en_hexdecode <= i_wb_dat[0];
                end
        endcase

        // if (i_rst) begin
        //     ssd_en_hexdecode <= 0;

        //     ssd_values[0] <= 8'h0;
        //     ssd_values[1] <= 8'h0;
        //     ssd_values[2] <= 8'h0;
        //     ssd_values[3] <= 8'h0;
        //     ssd_values[4] <= 8'h0;
        //     ssd_values[5] <= 8'h0;
        //     ssd_values[6] <= 8'h0;
        //     ssd_values[7] <= 8'h0;

        //     o_anode <= 8'b01111111;

        // end
    end

    // Controls 7seg outputs
    // We pulse each 7seg with its value for 1/8 of the time
    always @(*) begin 
        // we are only drawing one ssd value at a time

        case (ssd_current_value[3:0])        //SSD hex decode table
            4'h0: SSDhex = 8'b11000000; //0
            4'h1: SSDhex = 8'b11111001; //1
            4'h2: SSDhex = 8'b10100100; //2
            4'h3: SSDhex = 8'b10110000; //3
            4'h4: SSDhex = 8'b10011001; //4
            4'h5: SSDhex = 8'b10010010; //5
            4'h6: SSDhex = 8'b10000010; //6
            4'h7: SSDhex = 8'b11111000; //7
            4'h8: SSDhex = 8'b10000000; //8
            4'h9: SSDhex = 8'b10010000; //9
            4'hA: SSDhex = 8'b10001000; //A
            4'hB: SSDhex = 8'b10000011; //B
            4'hC: SSDhex = 8'b11000110; //C
            4'hD: SSDhex = 8'b10100001; //D
            4'hE: SSDhex = 8'b10000110; //E
            4'hF: SSDhex = 8'b10001110; //F
        endcase

        // case(index)                         //each anode is active low
        //     3'b000 : o_anode = 8'b01111111;
        //     3'b001 : o_anode = 8'b10111111;
        //     3'b010 : o_anode = 8'b11011111;
        //     3'b011 : o_anode = 8'b11101111;
        //     3'b100 : o_anode = 8'b11110111;
        //     3'b101 : o_anode = 8'b11111011;
        //     3'b110 : o_anode = 8'b11111101;
        //     3'b111 : o_anode = 8'b11111110;
        // endcase
        o_anode = ~(8'h1 << index);

    end
    
endmodule