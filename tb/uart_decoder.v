module uart_decoder
  #(parameter BAUD_RATE)
   (input rx);

   localparam T = 1000000000/BAUD_RATE;

   integer i;
   reg [7:0] ch;

   initial forever begin
      @(negedge rx);
      #(T/2) ch = 0;
      for (i=0;i<8;i=i+1)
	#T ch[i] = rx;
      $write("%c",ch);
      $fflush;
   end
endmodule