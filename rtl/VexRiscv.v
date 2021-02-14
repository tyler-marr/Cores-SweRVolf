// Generator : SpinalHDL v1.4.0    git head : ecb5a80b713566f417ea3ea061f9969e73770a7f
// Date      : 10/02/2021, 16:36:10
// Component : VexRiscv


`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10

`define EnvCtrlEnum_defaultEncoding_type [1:0]
`define EnvCtrlEnum_defaultEncoding_NONE 2'b00
`define EnvCtrlEnum_defaultEncoding_XRET 2'b01
`define EnvCtrlEnum_defaultEncoding_WFI 2'b10
`define EnvCtrlEnum_defaultEncoding_ECALL 2'b11

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define MmuPlugin_shared_State_defaultEncoding_type [2:0]
`define MmuPlugin_shared_State_defaultEncoding_IDLE 3'b000
`define MmuPlugin_shared_State_defaultEncoding_L1_CMD 3'b001
`define MmuPlugin_shared_State_defaultEncoding_L1_RSP 3'b010
`define MmuPlugin_shared_State_defaultEncoding_L0_CMD 3'b011
`define MmuPlugin_shared_State_defaultEncoding_L0_RSP 3'b100

`define JtagState_defaultEncoding_type [3:0]
`define JtagState_defaultEncoding_RESET 4'b0000
`define JtagState_defaultEncoding_IDLE 4'b0001
`define JtagState_defaultEncoding_IR_SELECT 4'b0010
`define JtagState_defaultEncoding_IR_CAPTURE 4'b0011
`define JtagState_defaultEncoding_IR_SHIFT 4'b0100
`define JtagState_defaultEncoding_IR_EXIT1 4'b0101
`define JtagState_defaultEncoding_IR_PAUSE 4'b0110
`define JtagState_defaultEncoding_IR_EXIT2 4'b0111
`define JtagState_defaultEncoding_IR_UPDATE 4'b1000
`define JtagState_defaultEncoding_DR_SELECT 4'b1001
`define JtagState_defaultEncoding_DR_CAPTURE 4'b1010
`define JtagState_defaultEncoding_DR_SHIFT 4'b1011
`define JtagState_defaultEncoding_DR_EXIT1 4'b1100
`define JtagState_defaultEncoding_DR_PAUSE 4'b1101
`define JtagState_defaultEncoding_DR_EXIT2 4'b1110
`define JtagState_defaultEncoding_DR_UPDATE 4'b1111


module BufferCC (
  input               io_dataIn,
  output              io_dataOut,
  input               clk,
  input               debugReset 
);
  reg                 buffers_0;
  reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

module FlowCCByToggle (
  input               io_input_valid,
  input               io_input_payload_last,
  input      [0:0]    io_input_payload_fragment,
  output              io_output_valid,
  output              io_output_payload_last,
  output     [0:0]    io_output_payload_fragment,
  input               io_jtag_tck,
  input               clk,
  input               debugReset 
);
  wire                inputArea_target_buffercc_io_dataOut;
  wire                outHitSignal;
  reg                 inputArea_target = 0;
  reg                 inputArea_data_last;
  reg        [0:0]    inputArea_data_fragment;
  wire                outputArea_target;
  reg                 outputArea_hit;
  wire                outputArea_flow_valid;
  wire                outputArea_flow_payload_last;
  wire       [0:0]    outputArea_flow_payload_fragment;
  reg                 outputArea_flow_regNext_valid;
  reg                 outputArea_flow_regNext_payload_last;
  reg        [0:0]    outputArea_flow_regNext_payload_fragment;

  BufferCC inputArea_target_buffercc ( 
    .io_dataIn     (inputArea_target                      ), //i
    .io_dataOut    (inputArea_target_buffercc_io_dataOut  ), //o
    .clk           (clk                                   ), //i
    .debugReset    (debugReset                            )  //i
  );
  assign outputArea_target = inputArea_target_buffercc_io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_last = inputArea_data_last;
  assign outputArea_flow_payload_fragment = inputArea_data_fragment;
  assign io_output_valid = outputArea_flow_regNext_valid;
  assign io_output_payload_last = outputArea_flow_regNext_payload_last;
  assign io_output_payload_fragment = outputArea_flow_regNext_payload_fragment;
  always @ (posedge io_jtag_tck) begin
    if(io_input_valid)begin
      inputArea_target <= (! inputArea_target);
      inputArea_data_last <= io_input_payload_last;
      inputArea_data_fragment <= io_input_payload_fragment;
    end
  end

  always @ (posedge clk) begin
    outputArea_hit <= outputArea_target;
    outputArea_flow_regNext_payload_last <= outputArea_flow_payload_last;
    outputArea_flow_regNext_payload_fragment <= outputArea_flow_payload_fragment;
  end

  always @ (posedge clk or posedge debugReset) begin
    if (debugReset) begin
      outputArea_flow_regNext_valid <= 1'b0;
    end else begin
      outputArea_flow_regNext_valid <= outputArea_flow_valid;
    end
  end


endmodule

module StreamFifoLowLatency (
  input               io_push_valid,
  output              io_push_ready,
  input               io_push_payload_error,
  input      [31:0]   io_push_payload_inst,
  output reg          io_pop_valid,
  input               io_pop_ready,
  output reg          io_pop_payload_error,
  output reg [31:0]   io_pop_payload_inst,
  input               io_flush,
  output     [0:0]    io_occupancy,
  input               clk,
  input               reset 
);
  wire                _zz_4_;
  wire       [0:0]    _zz_5_;
  reg                 _zz_1_;
  reg                 pushPtr_willIncrement;
  reg                 pushPtr_willClear;
  wire                pushPtr_willOverflowIfInc;
  wire                pushPtr_willOverflow;
  reg                 popPtr_willIncrement;
  reg                 popPtr_willClear;
  wire                popPtr_willOverflowIfInc;
  wire                popPtr_willOverflow;
  wire                ptrMatch;
  reg                 risingOccupancy;
  wire                empty;
  wire                full;
  wire                pushing;
  wire                popping;
  wire       [32:0]   _zz_2_;
  reg        [32:0]   _zz_3_;

  assign _zz_4_ = (! empty);
  assign _zz_5_ = _zz_2_[0 : 0];
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(pushing)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = 1'b1;
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      popPtr_willClear = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = 1'b1;
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  assign ptrMatch = 1'b1;
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if(_zz_4_)begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign _zz_2_ = _zz_3_;
  always @ (*) begin
    if(_zz_4_)begin
      io_pop_payload_error = _zz_5_[0];
    end else begin
      io_pop_payload_error = io_push_payload_error;
    end
  end

  always @ (*) begin
    if(_zz_4_)begin
      io_pop_payload_inst = _zz_2_[32 : 1];
    end else begin
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign io_occupancy = (risingOccupancy && ptrMatch);
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      risingOccupancy <= 1'b0;
    end else begin
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

  always @ (posedge clk) begin
    if(_zz_1_)begin
      _zz_3_ <= {io_push_payload_inst,io_push_payload_error};
    end
  end


endmodule

module JtagBridge (
  input               io_jtag_tms,
  input               io_jtag_tdi,
  output              io_jtag_tdo,
  input               io_jtag_tck,
  output              io_remote_cmd_valid,
  input               io_remote_cmd_ready,
  output              io_remote_cmd_payload_last,
  output     [0:0]    io_remote_cmd_payload_fragment,
  input               io_remote_rsp_valid,
  output              io_remote_rsp_ready,
  input               io_remote_rsp_payload_error,
  input      [31:0]   io_remote_rsp_payload_data,
  input               clk,
  input               debugReset 
);
  wire                flowCCByToggle_1__io_output_valid;
  wire                flowCCByToggle_1__io_output_payload_last;
  wire       [0:0]    flowCCByToggle_1__io_output_payload_fragment;
  wire                _zz_2_;
  wire                _zz_3_;
  wire       [0:0]    _zz_4_;
  wire       [3:0]    _zz_5_;
  wire       [1:0]    _zz_6_;
  wire       [3:0]    _zz_7_;
  wire       [1:0]    _zz_8_;
  wire       [3:0]    _zz_9_;
  wire       [0:0]    _zz_10_;
  wire                system_cmd_valid;
  wire                system_cmd_payload_last;
  wire       [0:0]    system_cmd_payload_fragment;
  reg                 system_rsp_valid;
  reg                 system_rsp_payload_error;
  reg        [31:0]   system_rsp_payload_data;
  wire       `JtagState_defaultEncoding_type jtag_tap_fsm_stateNext;
  reg        `JtagState_defaultEncoding_type jtag_tap_fsm_state = `JtagState_defaultEncoding_RESET;
  reg        `JtagState_defaultEncoding_type _zz_1_;
  reg        [3:0]    jtag_tap_instruction;
  reg        [3:0]    jtag_tap_instructionShift;
  reg                 jtag_tap_bypass;
  reg                 jtag_tap_tdoUnbufferd;
  reg                 jtag_tap_tdoUnbufferd_regNext;
  wire                jtag_idcodeArea_instructionHit;
  reg        [31:0]   jtag_idcodeArea_shifter;
  wire                jtag_writeArea_instructionHit;
  reg                 jtag_writeArea_source_valid;
  wire                jtag_writeArea_source_payload_last;
  wire       [0:0]    jtag_writeArea_source_payload_fragment;
  wire                jtag_readArea_instructionHit;
  reg        [33:0]   jtag_readArea_shifter;
  `ifndef SYNTHESIS
  reg [79:0] jtag_tap_fsm_stateNext_string;
  reg [79:0] jtag_tap_fsm_state_string;
  reg [79:0] _zz_1__string;
  `endif


  assign _zz_2_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_3_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_4_ = (1'b1);
  assign _zz_5_ = {3'd0, _zz_4_};
  assign _zz_6_ = (2'b10);
  assign _zz_7_ = {2'd0, _zz_6_};
  assign _zz_8_ = (2'b11);
  assign _zz_9_ = {2'd0, _zz_8_};
  assign _zz_10_ = (1'b1);
  FlowCCByToggle flowCCByToggle_1_ ( 
    .io_input_valid                (jtag_writeArea_source_valid                   ), //i
    .io_input_payload_last         (jtag_writeArea_source_payload_last            ), //i
    .io_input_payload_fragment     (jtag_writeArea_source_payload_fragment        ), //i
    .io_output_valid               (flowCCByToggle_1__io_output_valid             ), //o
    .io_output_payload_last        (flowCCByToggle_1__io_output_payload_last      ), //o
    .io_output_payload_fragment    (flowCCByToggle_1__io_output_payload_fragment  ), //o
    .io_jtag_tck                   (io_jtag_tck                                   ), //i
    .clk                           (clk                                           ), //i
    .debugReset                    (debugReset                                    )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(jtag_tap_fsm_stateNext)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_stateNext_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_stateNext_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_stateNext_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_stateNext_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_stateNext_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_stateNext_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_stateNext_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_stateNext_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_stateNext_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_stateNext_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_stateNext_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_stateNext_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_stateNext_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_stateNext_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_stateNext_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_stateNext_string = "DR_UPDATE ";
      default : jtag_tap_fsm_stateNext_string = "??????????";
    endcase
  end
  always @(*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_state_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_state_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_state_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_state_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_state_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_state_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_state_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_state_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_state_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_state_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_state_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_state_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_state_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_state_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_state_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_state_string = "DR_UPDATE ";
      default : jtag_tap_fsm_state_string = "??????????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `JtagState_defaultEncoding_RESET : _zz_1__string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : _zz_1__string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : _zz_1__string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : _zz_1__string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : _zz_1__string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : _zz_1__string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : _zz_1__string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : _zz_1__string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : _zz_1__string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : _zz_1__string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : _zz_1__string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : _zz_1__string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : _zz_1__string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : _zz_1__string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : _zz_1__string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : _zz_1__string = "DR_UPDATE ";
      default : _zz_1__string = "??????????";
    endcase
  end
  `endif

  assign io_remote_cmd_valid = system_cmd_valid;
  assign io_remote_cmd_payload_last = system_cmd_payload_last;
  assign io_remote_cmd_payload_fragment = system_cmd_payload_fragment;
  assign io_remote_rsp_ready = 1'b1;
  always @ (*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IDLE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_IR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IR_CAPTURE);
      end
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT2 : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_DR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_SELECT : `JtagState_defaultEncoding_DR_CAPTURE);
      end
      `JtagState_defaultEncoding_DR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT2 : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      default : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IDLE);
      end
    endcase
  end

  assign jtag_tap_fsm_stateNext = _zz_1_;
  always @ (*) begin
    jtag_tap_tdoUnbufferd = jtag_tap_bypass;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_tdoUnbufferd = jtag_tap_instructionShift[0];
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_tap_tdoUnbufferd = jtag_idcodeArea_shifter[0];
      end
    end
    if(jtag_readArea_instructionHit)begin
      if(_zz_3_)begin
        jtag_tap_tdoUnbufferd = jtag_readArea_shifter[0];
      end
    end
  end

  assign io_jtag_tdo = jtag_tap_tdoUnbufferd_regNext;
  assign jtag_idcodeArea_instructionHit = (jtag_tap_instruction == _zz_5_);
  assign jtag_writeArea_instructionHit = (jtag_tap_instruction == _zz_7_);
  always @ (*) begin
    jtag_writeArea_source_valid = 1'b0;
    if(jtag_writeArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT))begin
        jtag_writeArea_source_valid = 1'b1;
      end
    end
  end

  assign jtag_writeArea_source_payload_last = io_jtag_tms;
  assign jtag_writeArea_source_payload_fragment[0] = io_jtag_tdi;
  assign system_cmd_valid = flowCCByToggle_1__io_output_valid;
  assign system_cmd_payload_last = flowCCByToggle_1__io_output_payload_last;
  assign system_cmd_payload_fragment = flowCCByToggle_1__io_output_payload_fragment;
  assign jtag_readArea_instructionHit = (jtag_tap_instruction == _zz_9_);
  always @ (posedge clk) begin
    if(io_remote_cmd_valid)begin
      system_rsp_valid <= 1'b0;
    end
    if((io_remote_rsp_valid && io_remote_rsp_ready))begin
      system_rsp_valid <= 1'b1;
      system_rsp_payload_error <= io_remote_rsp_payload_error;
      system_rsp_payload_data <= io_remote_rsp_payload_data;
    end
  end

  always @ (posedge io_jtag_tck) begin
    jtag_tap_fsm_state <= jtag_tap_fsm_stateNext;
    jtag_tap_bypass <= io_jtag_tdi;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        jtag_tap_instructionShift <= jtag_tap_instruction;
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_instructionShift <= ({io_jtag_tdi,jtag_tap_instructionShift} >>> 1);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        jtag_tap_instruction <= jtag_tap_instructionShift;
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_idcodeArea_shifter <= ({io_jtag_tdi,jtag_idcodeArea_shifter} >>> 1);
      end
    end
    if((jtag_tap_fsm_state == `JtagState_defaultEncoding_RESET))begin
      jtag_idcodeArea_shifter <= 32'h10001fff;
      jtag_tap_instruction <= {3'd0, _zz_10_};
    end
    if(jtag_readArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_CAPTURE))begin
        jtag_readArea_shifter <= {{system_rsp_payload_data,system_rsp_payload_error},system_rsp_valid};
      end
      if(_zz_3_)begin
        jtag_readArea_shifter <= ({io_jtag_tdi,jtag_readArea_shifter} >>> 1);
      end
    end
  end

  always @ (negedge io_jtag_tck) begin
    jtag_tap_tdoUnbufferd_regNext <= jtag_tap_tdoUnbufferd;
  end


endmodule

module SystemDebugger (
  input               io_remote_cmd_valid,
  output              io_remote_cmd_ready,
  input               io_remote_cmd_payload_last,
  input      [0:0]    io_remote_cmd_payload_fragment,
  output              io_remote_rsp_valid,
  input               io_remote_rsp_ready,
  output              io_remote_rsp_payload_error,
  output     [31:0]   io_remote_rsp_payload_data,
  output              io_mem_cmd_valid,
  input               io_mem_cmd_ready,
  output     [31:0]   io_mem_cmd_payload_address,
  output     [31:0]   io_mem_cmd_payload_data,
  output              io_mem_cmd_payload_wr,
  output     [1:0]    io_mem_cmd_payload_size,
  input               io_mem_rsp_valid,
  input      [31:0]   io_mem_rsp_payload,
  input               clk,
  input               debugReset 
);
  wire                _zz_2_;
  wire       [0:0]    _zz_3_;
  reg        [66:0]   dispatcher_dataShifter;
  reg                 dispatcher_dataLoaded;
  reg        [7:0]    dispatcher_headerShifter;
  wire       [7:0]    dispatcher_header;
  reg                 dispatcher_headerLoaded;
  reg        [2:0]    dispatcher_counter;
  wire       [66:0]   _zz_1_;

  assign _zz_2_ = (dispatcher_headerLoaded == 1'b0);
  assign _zz_3_ = _zz_1_[64 : 64];
  assign dispatcher_header = dispatcher_headerShifter[7 : 0];
  assign io_remote_cmd_ready = (! dispatcher_dataLoaded);
  assign _zz_1_ = dispatcher_dataShifter[66 : 0];
  assign io_mem_cmd_payload_address = _zz_1_[31 : 0];
  assign io_mem_cmd_payload_data = _zz_1_[63 : 32];
  assign io_mem_cmd_payload_wr = _zz_3_[0];
  assign io_mem_cmd_payload_size = _zz_1_[66 : 65];
  assign io_mem_cmd_valid = (dispatcher_dataLoaded && (dispatcher_header == 8'h0));
  assign io_remote_rsp_valid = io_mem_rsp_valid;
  assign io_remote_rsp_payload_error = 1'b0;
  assign io_remote_rsp_payload_data = io_mem_rsp_payload;
  always @ (posedge clk or posedge debugReset) begin
    if (debugReset) begin
      dispatcher_dataLoaded <= 1'b0;
      dispatcher_headerLoaded <= 1'b0;
      dispatcher_counter <= (3'b000);
    end else begin
      if(io_remote_cmd_valid)begin
        if(_zz_2_)begin
          dispatcher_counter <= (dispatcher_counter + (3'b001));
          if((dispatcher_counter == (3'b111)))begin
            dispatcher_headerLoaded <= 1'b1;
          end
        end
        if(io_remote_cmd_payload_last)begin
          dispatcher_headerLoaded <= 1'b1;
          dispatcher_dataLoaded <= 1'b1;
          dispatcher_counter <= (3'b000);
        end
      end
      if((io_mem_cmd_valid && io_mem_cmd_ready))begin
        dispatcher_headerLoaded <= 1'b0;
        dispatcher_dataLoaded <= 1'b0;
      end
    end
  end

  always @ (posedge clk) begin
    if(io_remote_cmd_valid)begin
      if(_zz_2_)begin
        dispatcher_headerShifter <= ({io_remote_cmd_payload_fragment,dispatcher_headerShifter} >>> 1);
      end else begin
        dispatcher_dataShifter <= ({io_remote_cmd_payload_fragment,dispatcher_dataShifter} >>> 1);
      end
    end
  end


endmodule

module VexRiscv (
  input               timerInterrupt,
  input               externalInterrupt,
  input               softwareInterrupt,
  input               externalInterruptS,
  output              debug_resetOut,
  output              iBusWishbone_CYC,
  output              iBusWishbone_STB,
  input               iBusWishbone_ACK,
  output              iBusWishbone_WE,
  output     [29:0]   iBusWishbone_ADR,
  input      [31:0]   iBusWishbone_DAT_MISO,
  output     [31:0]   iBusWishbone_DAT_MOSI,
  output     [3:0]    iBusWishbone_SEL,
  input               iBusWishbone_ERR,
  output     [1:0]    iBusWishbone_BTE,
  output     [2:0]    iBusWishbone_CTI,
  output              dBusWishbone_CYC,
  output              dBusWishbone_STB,
  input               dBusWishbone_ACK,
  output              dBusWishbone_WE,
  output     [29:0]   dBusWishbone_ADR,
  input      [31:0]   dBusWishbone_DAT_MISO,
  output     [31:0]   dBusWishbone_DAT_MOSI,
  output reg [3:0]    dBusWishbone_SEL,
  input               dBusWishbone_ERR,
  output     [1:0]    dBusWishbone_BTE,
  output     [2:0]    dBusWishbone_CTI,
  input               jtag_tms,
  input               jtag_tdi,
  output              jtag_tdo,
  input               jtag_tck,
  input               clk,
  input               reset,
  input               debugReset 
);
  wire                _zz_194_;
  wire                _zz_195_;
  reg        [54:0]   _zz_196_;
  reg        [31:0]   _zz_197_;
  reg        [31:0]   _zz_198_;
  reg        [31:0]   _zz_199_;
  reg                 _zz_200_;
  reg                 _zz_201_;
  reg                 _zz_202_;
  reg        [9:0]    _zz_203_;
  reg        [9:0]    _zz_204_;
  reg        [9:0]    _zz_205_;
  reg        [9:0]    _zz_206_;
  reg                 _zz_207_;
  reg                 _zz_208_;
  reg                 _zz_209_;
  reg                 _zz_210_;
  reg                 _zz_211_;
  reg                 _zz_212_;
  reg                 _zz_213_;
  reg        [9:0]    _zz_214_;
  reg        [9:0]    _zz_215_;
  reg        [9:0]    _zz_216_;
  reg        [9:0]    _zz_217_;
  reg                 _zz_218_;
  reg                 _zz_219_;
  reg                 _zz_220_;
  reg                 _zz_221_;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  wire       [0:0]    IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy;
  wire                jtagBridge_1__io_jtag_tdo;
  wire                jtagBridge_1__io_remote_cmd_valid;
  wire                jtagBridge_1__io_remote_cmd_payload_last;
  wire       [0:0]    jtagBridge_1__io_remote_cmd_payload_fragment;
  wire                jtagBridge_1__io_remote_rsp_ready;
  wire                systemDebugger_1__io_remote_cmd_ready;
  wire                systemDebugger_1__io_remote_rsp_valid;
  wire                systemDebugger_1__io_remote_rsp_payload_error;
  wire       [31:0]   systemDebugger_1__io_remote_rsp_payload_data;
  wire                systemDebugger_1__io_mem_cmd_valid;
  wire       [31:0]   systemDebugger_1__io_mem_cmd_payload_address;
  wire       [31:0]   systemDebugger_1__io_mem_cmd_payload_data;
  wire                systemDebugger_1__io_mem_cmd_payload_wr;
  wire       [1:0]    systemDebugger_1__io_mem_cmd_payload_size;
  wire                _zz_222_;
  wire                _zz_223_;
  wire                _zz_224_;
  wire                _zz_225_;
  wire                _zz_226_;
  wire                _zz_227_;
  wire                _zz_228_;
  wire                _zz_229_;
  wire                _zz_230_;
  wire                _zz_231_;
  wire                _zz_232_;
  wire                _zz_233_;
  wire                _zz_234_;
  wire       [1:0]    _zz_235_;
  wire                _zz_236_;
  wire                _zz_237_;
  wire                _zz_238_;
  wire                _zz_239_;
  wire                _zz_240_;
  wire                _zz_241_;
  wire                _zz_242_;
  wire                _zz_243_;
  wire                _zz_244_;
  wire                _zz_245_;
  wire                _zz_246_;
  wire                _zz_247_;
  wire       [1:0]    _zz_248_;
  wire                _zz_249_;
  wire                _zz_250_;
  wire                _zz_251_;
  wire                _zz_252_;
  wire                _zz_253_;
  wire       [5:0]    _zz_254_;
  wire                _zz_255_;
  wire                _zz_256_;
  wire                _zz_257_;
  wire                _zz_258_;
  wire                _zz_259_;
  wire                _zz_260_;
  wire                _zz_261_;
  wire                _zz_262_;
  wire                _zz_263_;
  wire                _zz_264_;
  wire                _zz_265_;
  wire                _zz_266_;
  wire                _zz_267_;
  wire                _zz_268_;
  wire                _zz_269_;
  wire                _zz_270_;
  wire                _zz_271_;
  wire                _zz_272_;
  wire                _zz_273_;
  wire                _zz_274_;
  wire                _zz_275_;
  wire                _zz_276_;
  wire                _zz_277_;
  wire                _zz_278_;
  wire                _zz_279_;
  wire                _zz_280_;
  wire       [4:0]    _zz_281_;
  wire       [1:0]    _zz_282_;
  wire       [1:0]    _zz_283_;
  wire       [1:0]    _zz_284_;
  wire       [1:0]    _zz_285_;
  wire                _zz_286_;
  wire       [0:0]    _zz_287_;
  wire       [0:0]    _zz_288_;
  wire       [0:0]    _zz_289_;
  wire       [0:0]    _zz_290_;
  wire       [0:0]    _zz_291_;
  wire       [0:0]    _zz_292_;
  wire       [0:0]    _zz_293_;
  wire       [0:0]    _zz_294_;
  wire       [51:0]   _zz_295_;
  wire       [51:0]   _zz_296_;
  wire       [51:0]   _zz_297_;
  wire       [32:0]   _zz_298_;
  wire       [51:0]   _zz_299_;
  wire       [49:0]   _zz_300_;
  wire       [51:0]   _zz_301_;
  wire       [49:0]   _zz_302_;
  wire       [51:0]   _zz_303_;
  wire       [2:0]    _zz_304_;
  wire       [31:0]   _zz_305_;
  wire       [32:0]   _zz_306_;
  wire       [31:0]   _zz_307_;
  wire       [32:0]   _zz_308_;
  wire       [0:0]    _zz_309_;
  wire       [0:0]    _zz_310_;
  wire       [0:0]    _zz_311_;
  wire       [0:0]    _zz_312_;
  wire       [0:0]    _zz_313_;
  wire       [0:0]    _zz_314_;
  wire       [0:0]    _zz_315_;
  wire       [0:0]    _zz_316_;
  wire       [0:0]    _zz_317_;
  wire       [0:0]    _zz_318_;
  wire       [3:0]    _zz_319_;
  wire       [2:0]    _zz_320_;
  wire       [31:0]   _zz_321_;
  wire       [2:0]    _zz_322_;
  wire       [31:0]   _zz_323_;
  wire       [31:0]   _zz_324_;
  wire       [11:0]   _zz_325_;
  wire       [11:0]   _zz_326_;
  wire       [2:0]    _zz_327_;
  wire       [31:0]   _zz_328_;
  wire       [9:0]    _zz_329_;
  wire       [0:0]    _zz_330_;
  wire       [29:0]   _zz_331_;
  wire       [9:0]    _zz_332_;
  wire       [19:0]   _zz_333_;
  wire       [1:0]    _zz_334_;
  wire       [0:0]    _zz_335_;
  wire       [1:0]    _zz_336_;
  wire       [0:0]    _zz_337_;
  wire       [1:0]    _zz_338_;
  wire       [1:0]    _zz_339_;
  wire       [0:0]    _zz_340_;
  wire       [1:0]    _zz_341_;
  wire       [0:0]    _zz_342_;
  wire       [1:0]    _zz_343_;
  wire       [29:0]   _zz_344_;
  wire       [2:0]    _zz_345_;
  wire       [0:0]    _zz_346_;
  wire       [2:0]    _zz_347_;
  wire       [0:0]    _zz_348_;
  wire       [2:0]    _zz_349_;
  wire       [0:0]    _zz_350_;
  wire       [2:0]    _zz_351_;
  wire       [0:0]    _zz_352_;
  wire       [2:0]    _zz_353_;
  wire       [2:0]    _zz_354_;
  wire       [0:0]    _zz_355_;
  wire       [0:0]    _zz_356_;
  wire       [2:0]    _zz_357_;
  wire       [4:0]    _zz_358_;
  wire       [11:0]   _zz_359_;
  wire       [11:0]   _zz_360_;
  wire       [31:0]   _zz_361_;
  wire       [31:0]   _zz_362_;
  wire       [31:0]   _zz_363_;
  wire       [31:0]   _zz_364_;
  wire       [31:0]   _zz_365_;
  wire       [31:0]   _zz_366_;
  wire       [31:0]   _zz_367_;
  wire       [65:0]   _zz_368_;
  wire       [65:0]   _zz_369_;
  wire       [31:0]   _zz_370_;
  wire       [31:0]   _zz_371_;
  wire       [0:0]    _zz_372_;
  wire       [5:0]    _zz_373_;
  wire       [32:0]   _zz_374_;
  wire       [31:0]   _zz_375_;
  wire       [31:0]   _zz_376_;
  wire       [32:0]   _zz_377_;
  wire       [32:0]   _zz_378_;
  wire       [32:0]   _zz_379_;
  wire       [32:0]   _zz_380_;
  wire       [0:0]    _zz_381_;
  wire       [32:0]   _zz_382_;
  wire       [0:0]    _zz_383_;
  wire       [32:0]   _zz_384_;
  wire       [0:0]    _zz_385_;
  wire       [31:0]   _zz_386_;
  wire       [1:0]    _zz_387_;
  wire       [1:0]    _zz_388_;
  wire       [19:0]   _zz_389_;
  wire       [11:0]   _zz_390_;
  wire       [11:0]   _zz_391_;
  wire       [0:0]    _zz_392_;
  wire       [1:0]    _zz_393_;
  wire       [0:0]    _zz_394_;
  wire       [1:0]    _zz_395_;
  wire       [0:0]    _zz_396_;
  wire       [0:0]    _zz_397_;
  wire       [0:0]    _zz_398_;
  wire       [0:0]    _zz_399_;
  wire       [0:0]    _zz_400_;
  wire       [0:0]    _zz_401_;
  wire       [0:0]    _zz_402_;
  wire       [0:0]    _zz_403_;
  wire       [0:0]    _zz_404_;
  wire       [0:0]    _zz_405_;
  wire       [0:0]    _zz_406_;
  wire       [0:0]    _zz_407_;
  wire       [0:0]    _zz_408_;
  wire       [0:0]    _zz_409_;
  wire       [0:0]    _zz_410_;
  wire       [0:0]    _zz_411_;
  wire       [0:0]    _zz_412_;
  wire       [0:0]    _zz_413_;
  wire       [0:0]    _zz_414_;
  wire       [0:0]    _zz_415_;
  wire       [0:0]    _zz_416_;
  wire       [0:0]    _zz_417_;
  wire       [0:0]    _zz_418_;
  wire       [0:0]    _zz_419_;
  wire       [0:0]    _zz_420_;
  wire       [0:0]    _zz_421_;
  wire       [0:0]    _zz_422_;
  wire       [0:0]    _zz_423_;
  wire       [0:0]    _zz_424_;
  wire       [0:0]    _zz_425_;
  wire       [0:0]    _zz_426_;
  wire       [0:0]    _zz_427_;
  wire       [0:0]    _zz_428_;
  wire       [0:0]    _zz_429_;
  wire       [0:0]    _zz_430_;
  wire       [0:0]    _zz_431_;
  wire       [0:0]    _zz_432_;
  wire       [0:0]    _zz_433_;
  wire       [0:0]    _zz_434_;
  wire       [0:0]    _zz_435_;
  wire       [0:0]    _zz_436_;
  wire       [0:0]    _zz_437_;
  wire       [0:0]    _zz_438_;
  wire       [0:0]    _zz_439_;
  wire       [0:0]    _zz_440_;
  wire       [0:0]    _zz_441_;
  wire       [0:0]    _zz_442_;
  wire       [0:0]    _zz_443_;
  wire       [0:0]    _zz_444_;
  wire       [0:0]    _zz_445_;
  wire       [0:0]    _zz_446_;
  wire       [0:0]    _zz_447_;
  wire       [0:0]    _zz_448_;
  wire       [54:0]   _zz_449_;
  wire                _zz_450_;
  wire                _zz_451_;
  wire       [1:0]    _zz_452_;
  wire       [31:0]   _zz_453_;
  wire       [31:0]   _zz_454_;
  wire       [31:0]   _zz_455_;
  wire                _zz_456_;
  wire       [0:0]    _zz_457_;
  wire       [15:0]   _zz_458_;
  wire       [31:0]   _zz_459_;
  wire       [31:0]   _zz_460_;
  wire       [31:0]   _zz_461_;
  wire                _zz_462_;
  wire       [0:0]    _zz_463_;
  wire       [9:0]    _zz_464_;
  wire       [31:0]   _zz_465_;
  wire       [31:0]   _zz_466_;
  wire       [31:0]   _zz_467_;
  wire                _zz_468_;
  wire       [0:0]    _zz_469_;
  wire       [3:0]    _zz_470_;
  wire                _zz_471_;
  wire                _zz_472_;
  wire       [6:0]    _zz_473_;
  wire       [4:0]    _zz_474_;
  wire                _zz_475_;
  wire       [4:0]    _zz_476_;
  wire                _zz_477_;
  wire       [0:0]    _zz_478_;
  wire       [1:0]    _zz_479_;
  wire       [31:0]   _zz_480_;
  wire       [31:0]   _zz_481_;
  wire                _zz_482_;
  wire       [2:0]    _zz_483_;
  wire       [2:0]    _zz_484_;
  wire                _zz_485_;
  wire       [0:0]    _zz_486_;
  wire       [26:0]   _zz_487_;
  wire       [31:0]   _zz_488_;
  wire       [31:0]   _zz_489_;
  wire       [31:0]   _zz_490_;
  wire       [31:0]   _zz_491_;
  wire       [31:0]   _zz_492_;
  wire       [31:0]   _zz_493_;
  wire       [31:0]   _zz_494_;
  wire                _zz_495_;
  wire                _zz_496_;
  wire                _zz_497_;
  wire       [0:0]    _zz_498_;
  wire       [0:0]    _zz_499_;
  wire       [4:0]    _zz_500_;
  wire       [4:0]    _zz_501_;
  wire                _zz_502_;
  wire       [0:0]    _zz_503_;
  wire       [23:0]   _zz_504_;
  wire       [31:0]   _zz_505_;
  wire       [31:0]   _zz_506_;
  wire       [31:0]   _zz_507_;
  wire       [31:0]   _zz_508_;
  wire       [31:0]   _zz_509_;
  wire       [31:0]   _zz_510_;
  wire       [31:0]   _zz_511_;
  wire                _zz_512_;
  wire       [0:0]    _zz_513_;
  wire       [2:0]    _zz_514_;
  wire       [0:0]    _zz_515_;
  wire       [0:0]    _zz_516_;
  wire       [2:0]    _zz_517_;
  wire       [2:0]    _zz_518_;
  wire                _zz_519_;
  wire       [0:0]    _zz_520_;
  wire       [21:0]   _zz_521_;
  wire       [31:0]   _zz_522_;
  wire       [31:0]   _zz_523_;
  wire       [31:0]   _zz_524_;
  wire                _zz_525_;
  wire       [0:0]    _zz_526_;
  wire       [0:0]    _zz_527_;
  wire       [31:0]   _zz_528_;
  wire       [31:0]   _zz_529_;
  wire       [31:0]   _zz_530_;
  wire       [31:0]   _zz_531_;
  wire                _zz_532_;
  wire       [0:0]    _zz_533_;
  wire       [0:0]    _zz_534_;
  wire       [0:0]    _zz_535_;
  wire       [5:0]    _zz_536_;
  wire       [0:0]    _zz_537_;
  wire       [0:0]    _zz_538_;
  wire                _zz_539_;
  wire       [0:0]    _zz_540_;
  wire       [19:0]   _zz_541_;
  wire       [31:0]   _zz_542_;
  wire       [31:0]   _zz_543_;
  wire       [31:0]   _zz_544_;
  wire       [31:0]   _zz_545_;
  wire       [31:0]   _zz_546_;
  wire       [31:0]   _zz_547_;
  wire       [31:0]   _zz_548_;
  wire       [31:0]   _zz_549_;
  wire                _zz_550_;
  wire       [0:0]    _zz_551_;
  wire       [3:0]    _zz_552_;
  wire       [31:0]   _zz_553_;
  wire       [31:0]   _zz_554_;
  wire                _zz_555_;
  wire       [1:0]    _zz_556_;
  wire       [1:0]    _zz_557_;
  wire                _zz_558_;
  wire       [0:0]    _zz_559_;
  wire       [17:0]   _zz_560_;
  wire       [31:0]   _zz_561_;
  wire       [31:0]   _zz_562_;
  wire       [31:0]   _zz_563_;
  wire                _zz_564_;
  wire       [0:0]    _zz_565_;
  wire       [1:0]    _zz_566_;
  wire       [31:0]   _zz_567_;
  wire                _zz_568_;
  wire       [0:0]    _zz_569_;
  wire       [4:0]    _zz_570_;
  wire       [0:0]    _zz_571_;
  wire       [0:0]    _zz_572_;
  wire                _zz_573_;
  wire       [0:0]    _zz_574_;
  wire       [15:0]   _zz_575_;
  wire       [31:0]   _zz_576_;
  wire       [31:0]   _zz_577_;
  wire       [31:0]   _zz_578_;
  wire                _zz_579_;
  wire       [31:0]   _zz_580_;
  wire       [31:0]   _zz_581_;
  wire       [31:0]   _zz_582_;
  wire                _zz_583_;
  wire       [0:0]    _zz_584_;
  wire       [2:0]    _zz_585_;
  wire       [31:0]   _zz_586_;
  wire       [31:0]   _zz_587_;
  wire                _zz_588_;
  wire       [1:0]    _zz_589_;
  wire       [1:0]    _zz_590_;
  wire                _zz_591_;
  wire       [0:0]    _zz_592_;
  wire       [13:0]   _zz_593_;
  wire       [31:0]   _zz_594_;
  wire       [31:0]   _zz_595_;
  wire       [31:0]   _zz_596_;
  wire       [31:0]   _zz_597_;
  wire                _zz_598_;
  wire       [0:0]    _zz_599_;
  wire       [0:0]    _zz_600_;
  wire       [31:0]   _zz_601_;
  wire                _zz_602_;
  wire                _zz_603_;
  wire       [0:0]    _zz_604_;
  wire       [0:0]    _zz_605_;
  wire                _zz_606_;
  wire       [0:0]    _zz_607_;
  wire       [11:0]   _zz_608_;
  wire       [31:0]   _zz_609_;
  wire       [31:0]   _zz_610_;
  wire       [31:0]   _zz_611_;
  wire                _zz_612_;
  wire                _zz_613_;
  wire       [0:0]    _zz_614_;
  wire       [0:0]    _zz_615_;
  wire       [0:0]    _zz_616_;
  wire       [0:0]    _zz_617_;
  wire                _zz_618_;
  wire       [0:0]    _zz_619_;
  wire       [8:0]    _zz_620_;
  wire       [31:0]   _zz_621_;
  wire       [31:0]   _zz_622_;
  wire       [31:0]   _zz_623_;
  wire       [0:0]    _zz_624_;
  wire       [0:0]    _zz_625_;
  wire       [0:0]    _zz_626_;
  wire       [0:0]    _zz_627_;
  wire       [4:0]    _zz_628_;
  wire       [4:0]    _zz_629_;
  wire                _zz_630_;
  wire       [0:0]    _zz_631_;
  wire       [5:0]    _zz_632_;
  wire       [31:0]   _zz_633_;
  wire       [31:0]   _zz_634_;
  wire       [31:0]   _zz_635_;
  wire       [31:0]   _zz_636_;
  wire                _zz_637_;
  wire       [0:0]    _zz_638_;
  wire       [1:0]    _zz_639_;
  wire       [0:0]    _zz_640_;
  wire       [0:0]    _zz_641_;
  wire       [1:0]    _zz_642_;
  wire       [1:0]    _zz_643_;
  wire                _zz_644_;
  wire       [0:0]    _zz_645_;
  wire       [2:0]    _zz_646_;
  wire       [31:0]   _zz_647_;
  wire       [31:0]   _zz_648_;
  wire       [31:0]   _zz_649_;
  wire       [31:0]   _zz_650_;
  wire       [31:0]   _zz_651_;
  wire       [31:0]   _zz_652_;
  wire                _zz_653_;
  wire                _zz_654_;
  wire                _zz_655_;
  wire       [0:0]    _zz_656_;
  wire       [0:0]    _zz_657_;
  wire                _zz_658_;
  wire                _zz_659_;
  wire       [31:0]   _zz_660_;
  wire       [31:0]   _zz_661_;
  wire       [31:0]   _zz_662_;
  wire                decode_SRC2_FORCE_ZERO;
  wire                decode_DO_EBREAK;
  wire                memory_ATOMIC_HIT;
  wire       `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_1_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_2_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_3_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_4_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_5_;
  wire       `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_6_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_7_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_8_;
  wire       [31:0]   writeBack_REGFILE_WRITE_DATA;
  wire       [31:0]   execute_REGFILE_WRITE_DATA;
  wire       [31:0]   writeBack_FORMAL_PC_NEXT;
  wire       [31:0]   memory_FORMAL_PC_NEXT;
  wire       [31:0]   execute_FORMAL_PC_NEXT;
  wire       [31:0]   decode_FORMAL_PC_NEXT;
  wire       `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_9_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_10_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_11_;
  wire                memory_IS_SFENCE_VMA;
  wire                execute_IS_SFENCE_VMA;
  wire                decode_IS_SFENCE_VMA;
  wire       [33:0]   execute_MUL_LH;
  wire                decode_SRC_LESS_UNSIGNED;
  wire                decode_MEMORY_STORE;
  wire       [33:0]   memory_MUL_HH;
  wire       [33:0]   execute_MUL_HH;
  wire                memory_IS_MUL;
  wire                execute_IS_MUL;
  wire                decode_IS_MUL;
  wire                decode_IS_CSR;
  wire                execute_PREDICTION_CONTEXT_hazard;
  wire                execute_PREDICTION_CONTEXT_hit;
  wire       [19:0]   execute_PREDICTION_CONTEXT_line_source;
  wire       [1:0]    execute_PREDICTION_CONTEXT_line_branchWish;
  wire                execute_PREDICTION_CONTEXT_line_last2Bytes;
  wire       [31:0]   execute_PREDICTION_CONTEXT_line_target;
  wire                decode_PREDICTION_CONTEXT_hazard;
  wire                decode_PREDICTION_CONTEXT_hit;
  wire       [19:0]   decode_PREDICTION_CONTEXT_line_source;
  wire       [1:0]    decode_PREDICTION_CONTEXT_line_branchWish;
  wire                decode_PREDICTION_CONTEXT_line_last2Bytes;
  wire       [31:0]   decode_PREDICTION_CONTEXT_line_target;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_12_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_13_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_14_;
  wire                decode_BYPASSABLE_EXECUTE_STAGE;
  wire                execute_BYPASSABLE_MEMORY_STAGE;
  wire                decode_BYPASSABLE_MEMORY_STAGE;
  wire                memory_MEMORY_ATOMIC;
  wire                decode_MEMORY_ATOMIC;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_15_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_16_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_17_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_18_;
  wire       `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_19_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_20_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_21_;
  wire       [51:0]   memory_MUL_LOW;
  wire                decode_CSR_WRITE_OPCODE;
  wire       [31:0]   execute_NEXT_PC2;
  wire       [31:0]   execute_SHIFT_RIGHT;
  wire       [33:0]   execute_MUL_HL;
  wire       [1:0]    memory_MEMORY_ADDRESS_LOW;
  wire       [1:0]    execute_MEMORY_ADDRESS_LOW;
  wire                decode_IS_RS1_SIGNED;
  wire                decode_IS_RS2_SIGNED;
  wire                decode_CSR_READ_OPCODE;
  wire                execute_BRANCH_DO;
  wire                execute_TARGET_MISSMATCH2;
  wire       [31:0]   decode_SRC2;
  wire                decode_IS_DIV;
  wire       [31:0]   execute_MUL_LL;
  wire       [31:0]   decode_SRC1;
  wire       [31:0]   memory_MEMORY_READ_DATA;
  wire                writeBack_IS_SFENCE_VMA;
  wire       [31:0]   memory_NEXT_PC2;
  wire       [31:0]   memory_BRANCH_CALC;
  wire                memory_TARGET_MISSMATCH2;
  wire                memory_BRANCH_DO;
  wire       [31:0]   execute_BRANCH_CALC;
  wire                execute_IS_RVC;
  wire       [31:0]   execute_BRANCH_SRC22;
  wire       `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_22_;
  wire       [31:0]   execute_PC;
  wire                execute_DO_EBREAK;
  wire                decode_IS_EBREAK;
  wire                execute_CSR_READ_OPCODE;
  wire                execute_CSR_WRITE_OPCODE;
  wire                execute_IS_CSR;
  wire       `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_23_;
  wire       `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_24_;
  wire       `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_25_;
  wire                execute_IS_RS1_SIGNED;
  wire                execute_IS_DIV;
  wire                execute_IS_RS2_SIGNED;
  wire                memory_IS_DIV;
  wire                writeBack_IS_MUL;
  wire       [33:0]   writeBack_MUL_HH;
  wire       [51:0]   writeBack_MUL_LOW;
  wire       [33:0]   memory_MUL_HL;
  wire       [33:0]   memory_MUL_LH;
  wire       [31:0]   memory_MUL_LL;
  (* keep , syn_keep *) wire       [31:0]   execute_RS1 /* synthesis syn_keep = 1 */ ;
  wire                decode_RS2_USE;
  wire                decode_RS1_USE;
  reg        [31:0]   _zz_26_;
  wire                execute_REGFILE_WRITE_VALID;
  wire                execute_BYPASSABLE_EXECUTE_STAGE;
  wire                memory_REGFILE_WRITE_VALID;
  wire       [31:0]   memory_INSTRUCTION;
  wire                memory_BYPASSABLE_MEMORY_STAGE;
  wire                writeBack_REGFILE_WRITE_VALID;
  reg        [31:0]   decode_RS2;
  reg        [31:0]   decode_RS1;
  wire       [31:0]   memory_SHIFT_RIGHT;
  reg        [31:0]   _zz_27_;
  wire       `ShiftCtrlEnum_defaultEncoding_type memory_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_28_;
  wire       `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_29_;
  wire                execute_SRC_LESS_UNSIGNED;
  wire                execute_SRC2_FORCE_ZERO;
  wire                execute_SRC_USE_SUB_LESS;
  wire       [31:0]   _zz_30_;
  wire       [31:0]   _zz_31_;
  wire       `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_32_;
  wire       [31:0]   _zz_33_;
  wire       `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_34_;
  wire                decode_SRC_USE_SUB_LESS;
  wire                decode_SRC_ADD_ZERO;
  wire       [31:0]   execute_SRC_ADD_SUB;
  wire                execute_SRC_LESS;
  wire       `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_35_;
  wire       [31:0]   execute_SRC2;
  wire       [31:0]   execute_SRC1;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_36_;
  wire       [31:0]   _zz_37_;
  wire                _zz_38_;
  reg                 _zz_39_;
  wire       [31:0]   decode_INSTRUCTION_ANTICIPATED;
  reg                 decode_REGFILE_WRITE_VALID;
  wire                decode_LEGAL_INSTRUCTION;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_40_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_41_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_42_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_43_;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_44_;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_45_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_46_;
  wire                writeBack_ATOMIC_HIT;
  wire                writeBack_MEMORY_STORE;
  wire                writeBack_MEMORY_ATOMIC;
  reg        [31:0]   _zz_47_;
  wire                writeBack_MEMORY_ENABLE;
  wire       [1:0]    writeBack_MEMORY_ADDRESS_LOW;
  wire       [31:0]   writeBack_MEMORY_READ_DATA;
  wire                memory_MMU_FAULT;
  wire       [31:0]   memory_MMU_RSP_physicalAddress;
  wire                memory_MMU_RSP_isIoAccess;
  wire                memory_MMU_RSP_isPaging;
  wire                memory_MMU_RSP_allowRead;
  wire                memory_MMU_RSP_allowWrite;
  wire                memory_MMU_RSP_allowExecute;
  wire                memory_MMU_RSP_exception;
  wire                memory_MMU_RSP_refilling;
  wire                memory_ALIGNEMENT_FAULT;
  wire       [31:0]   memory_REGFILE_WRITE_DATA;
  wire                memory_MEMORY_STORE;
  wire                memory_MEMORY_ENABLE;
  wire                execute_ATOMIC_HIT;
  wire                execute_MEMORY_ATOMIC;
  wire                execute_MMU_FAULT;
  wire       [31:0]   execute_MMU_RSP_physicalAddress;
  wire                execute_MMU_RSP_isIoAccess;
  wire                execute_MMU_RSP_isPaging;
  wire                execute_MMU_RSP_allowRead;
  wire                execute_MMU_RSP_allowWrite;
  wire                execute_MMU_RSP_allowExecute;
  wire                execute_MMU_RSP_exception;
  wire                execute_MMU_RSP_refilling;
  wire       [31:0]   execute_SRC_ADD;
  (* keep , syn_keep *) wire       [31:0]   execute_RS2 /* synthesis syn_keep = 1 */ ;
  wire       [31:0]   execute_INSTRUCTION;
  wire                execute_MEMORY_STORE;
  wire                execute_MEMORY_ENABLE;
  wire                execute_ALIGNEMENT_FAULT;
  wire                decode_MEMORY_ENABLE;
  wire                memory_IS_RVC;
  wire       [31:0]   memory_PC;
  wire                memory_PREDICTION_CONTEXT_hazard;
  wire                memory_PREDICTION_CONTEXT_hit;
  wire       [19:0]   memory_PREDICTION_CONTEXT_line_source;
  wire       [1:0]    memory_PREDICTION_CONTEXT_line_branchWish;
  wire                memory_PREDICTION_CONTEXT_line_last2Bytes;
  wire       [31:0]   memory_PREDICTION_CONTEXT_line_target;
  reg                 _zz_48_;
  reg        [31:0]   _zz_49_;
  reg        [31:0]   _zz_50_;
  wire       [31:0]   decode_PC;
  reg        [31:0]   _zz_51_;
  wire       [31:0]   decode_INSTRUCTION;
  wire                decode_IS_RVC;
  wire       [31:0]   writeBack_PC;
  wire       [31:0]   writeBack_INSTRUCTION;
  reg                 decode_arbitration_haltItself;
  reg                 decode_arbitration_haltByOther;
  reg                 decode_arbitration_removeIt;
  wire                decode_arbitration_flushIt;
  reg                 decode_arbitration_flushNext;
  reg                 decode_arbitration_isValid;
  wire                decode_arbitration_isStuck;
  wire                decode_arbitration_isStuckByOthers;
  wire                decode_arbitration_isFlushed;
  wire                decode_arbitration_isMoving;
  wire                decode_arbitration_isFiring;
  reg                 execute_arbitration_haltItself;
  reg                 execute_arbitration_haltByOther;
  reg                 execute_arbitration_removeIt;
  reg                 execute_arbitration_flushIt;
  reg                 execute_arbitration_flushNext;
  reg                 execute_arbitration_isValid;
  wire                execute_arbitration_isStuck;
  wire                execute_arbitration_isStuckByOthers;
  wire                execute_arbitration_isFlushed;
  wire                execute_arbitration_isMoving;
  wire                execute_arbitration_isFiring;
  reg                 memory_arbitration_haltItself;
  wire                memory_arbitration_haltByOther;
  reg                 memory_arbitration_removeIt;
  reg                 memory_arbitration_flushIt;
  reg                 memory_arbitration_flushNext;
  reg                 memory_arbitration_isValid;
  wire                memory_arbitration_isStuck;
  wire                memory_arbitration_isStuckByOthers;
  wire                memory_arbitration_isFlushed;
  wire                memory_arbitration_isMoving;
  wire                memory_arbitration_isFiring;
  wire                writeBack_arbitration_haltItself;
  wire                writeBack_arbitration_haltByOther;
  reg                 writeBack_arbitration_removeIt;
  wire                writeBack_arbitration_flushIt;
  reg                 writeBack_arbitration_flushNext;
  reg                 writeBack_arbitration_isValid;
  wire                writeBack_arbitration_isStuck;
  wire                writeBack_arbitration_isStuckByOthers;
  wire                writeBack_arbitration_isFlushed;
  wire                writeBack_arbitration_isMoving;
  wire                writeBack_arbitration_isFiring;
  wire       [31:0]   lastStageInstruction /* verilator public */ ;
  wire       [31:0]   lastStagePc /* verilator public */ ;
  wire                lastStageIsValid /* verilator public */ ;
  wire                lastStageIsFiring /* verilator public */ ;
  reg                 IBusSimplePlugin_fetcherHalt;
  reg                 IBusSimplePlugin_incomingInstruction;
  wire                IBusSimplePlugin_fetchPrediction_cmd_hadBranch;
  wire       [31:0]   IBusSimplePlugin_fetchPrediction_cmd_targetPc;
  wire                IBusSimplePlugin_fetchPrediction_rsp_wasRight;
  wire       [31:0]   IBusSimplePlugin_fetchPrediction_rsp_finalPc;
  wire       [31:0]   IBusSimplePlugin_fetchPrediction_rsp_sourceLastWord;
  wire                IBusSimplePlugin_pcValids_0;
  wire                IBusSimplePlugin_pcValids_1;
  wire                IBusSimplePlugin_pcValids_2;
  wire                IBusSimplePlugin_pcValids_3;
  wire                iBus_cmd_valid;
  wire                iBus_cmd_ready;
  wire       [31:0]   iBus_cmd_payload_pc;
  wire                iBus_rsp_valid;
  wire                iBus_rsp_payload_error;
  wire       [31:0]   iBus_rsp_payload_inst;
  wire                IBusSimplePlugin_decodeExceptionPort_valid;
  reg        [3:0]    IBusSimplePlugin_decodeExceptionPort_payload_code;
  wire       [31:0]   IBusSimplePlugin_decodeExceptionPort_payload_badAddr;
  wire                IBusSimplePlugin_mmuBus_cmd_isValid;
  wire       [31:0]   IBusSimplePlugin_mmuBus_cmd_virtualAddress;
  wire                IBusSimplePlugin_mmuBus_cmd_bypassTranslation;
  reg        [31:0]   IBusSimplePlugin_mmuBus_rsp_physicalAddress;
  wire                IBusSimplePlugin_mmuBus_rsp_isIoAccess;
  reg                 IBusSimplePlugin_mmuBus_rsp_isPaging;
  reg                 IBusSimplePlugin_mmuBus_rsp_allowRead;
  reg                 IBusSimplePlugin_mmuBus_rsp_allowWrite;
  reg                 IBusSimplePlugin_mmuBus_rsp_allowExecute;
  reg                 IBusSimplePlugin_mmuBus_rsp_exception;
  reg                 IBusSimplePlugin_mmuBus_rsp_refilling;
  wire                IBusSimplePlugin_mmuBus_end;
  wire                IBusSimplePlugin_mmuBus_busy;
  reg                 DBusSimplePlugin_memoryExceptionPort_valid;
  reg        [3:0]    DBusSimplePlugin_memoryExceptionPort_payload_code;
  wire       [31:0]   DBusSimplePlugin_memoryExceptionPort_payload_badAddr;
  wire                DBusSimplePlugin_mmuBus_cmd_isValid;
  wire       [31:0]   DBusSimplePlugin_mmuBus_cmd_virtualAddress;
  wire                DBusSimplePlugin_mmuBus_cmd_bypassTranslation;
  reg        [31:0]   DBusSimplePlugin_mmuBus_rsp_physicalAddress;
  wire                DBusSimplePlugin_mmuBus_rsp_isIoAccess;
  reg                 DBusSimplePlugin_mmuBus_rsp_isPaging;
  reg                 DBusSimplePlugin_mmuBus_rsp_allowRead;
  reg                 DBusSimplePlugin_mmuBus_rsp_allowWrite;
  reg                 DBusSimplePlugin_mmuBus_rsp_allowExecute;
  reg                 DBusSimplePlugin_mmuBus_rsp_exception;
  reg                 DBusSimplePlugin_mmuBus_rsp_refilling;
  wire                DBusSimplePlugin_mmuBus_end;
  wire                DBusSimplePlugin_mmuBus_busy;
  reg                 DBusSimplePlugin_redoBranch_valid;
  wire       [31:0]   DBusSimplePlugin_redoBranch_payload;
  wire                decodeExceptionPort_valid;
  wire       [3:0]    decodeExceptionPort_payload_code;
  wire       [31:0]   decodeExceptionPort_payload_badAddr;
  reg                 CsrPlugin_inWfi /* verilator public */ ;
  reg                 CsrPlugin_thirdPartyWake;
  reg                 CsrPlugin_jumpInterface_valid;
  reg        [31:0]   CsrPlugin_jumpInterface_payload;
  reg                 CsrPlugin_redoInterface_valid;
  wire       [31:0]   CsrPlugin_redoInterface_payload;
  wire                CsrPlugin_exceptionPendings_0;
  wire                CsrPlugin_exceptionPendings_1;
  wire                CsrPlugin_exceptionPendings_2;
  wire                CsrPlugin_exceptionPendings_3;
  wire                contextSwitching;
  reg        [1:0]    CsrPlugin_privilege;
  reg                 CsrPlugin_forceMachineWire;
  reg                 CsrPlugin_selfException_valid;
  reg        [3:0]    CsrPlugin_selfException_payload_code;
  wire       [31:0]   CsrPlugin_selfException_payload_badAddr;
  reg                 CsrPlugin_allowInterrupts;
  reg                 CsrPlugin_allowException;
  wire                debug_bus_cmd_valid;
  reg                 debug_bus_cmd_ready;
  wire                debug_bus_cmd_payload_wr;
  wire       [7:0]    debug_bus_cmd_payload_address;
  wire       [31:0]   debug_bus_cmd_payload_data;
  reg        [31:0]   debug_bus_rsp_data;
  reg                 IBusSimplePlugin_injectionPort_valid;
  reg                 IBusSimplePlugin_injectionPort_ready;
  wire       [31:0]   IBusSimplePlugin_injectionPort_payload;
  wire                BranchPlugin_jumpInterface_valid;
  wire       [31:0]   BranchPlugin_jumpInterface_payload;
  reg                 MmuPlugin_dBusAccess_cmd_valid;
  reg                 MmuPlugin_dBusAccess_cmd_ready;
  reg        [31:0]   MmuPlugin_dBusAccess_cmd_payload_address;
  wire       [1:0]    MmuPlugin_dBusAccess_cmd_payload_size;
  wire                MmuPlugin_dBusAccess_cmd_payload_write;
  wire       [31:0]   MmuPlugin_dBusAccess_cmd_payload_data;
  wire       [3:0]    MmuPlugin_dBusAccess_cmd_payload_writeMask;
  reg                 MmuPlugin_dBusAccess_rsp_valid;
  wire       [31:0]   MmuPlugin_dBusAccess_rsp_payload_data;
  wire                MmuPlugin_dBusAccess_rsp_payload_error;
  wire                MmuPlugin_dBusAccess_rsp_payload_redo;
  wire                IBusSimplePlugin_externalFlush;
  wire                IBusSimplePlugin_jump_pcLoad_valid;
  wire       [31:0]   IBusSimplePlugin_jump_pcLoad_payload;
  wire       [3:0]    _zz_52_;
  wire       [3:0]    _zz_53_;
  wire                _zz_54_;
  wire                _zz_55_;
  wire                _zz_56_;
  wire                IBusSimplePlugin_fetchPc_output_valid;
  wire                IBusSimplePlugin_fetchPc_output_ready;
  wire       [31:0]   IBusSimplePlugin_fetchPc_output_payload;
  reg        [31:0]   IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg                 IBusSimplePlugin_fetchPc_correction;
  reg                 IBusSimplePlugin_fetchPc_correctionReg;
  wire                IBusSimplePlugin_fetchPc_corrected;
  wire                IBusSimplePlugin_fetchPc_pcRegPropagate;
  reg                 IBusSimplePlugin_fetchPc_booted;
  reg                 IBusSimplePlugin_fetchPc_inc;
  reg        [31:0]   IBusSimplePlugin_fetchPc_pc;
  wire                IBusSimplePlugin_fetchPc_predictionPcLoad_valid;
  wire       [31:0]   IBusSimplePlugin_fetchPc_predictionPcLoad_payload;
  wire                IBusSimplePlugin_fetchPc_redo_valid;
  reg        [31:0]   IBusSimplePlugin_fetchPc_redo_payload;
  reg                 IBusSimplePlugin_fetchPc_flushed;
  reg                 IBusSimplePlugin_decodePc_flushed;
  reg        [31:0]   IBusSimplePlugin_decodePc_pcReg /* verilator public */ ;
  wire       [31:0]   IBusSimplePlugin_decodePc_pcPlus;
  reg                 IBusSimplePlugin_decodePc_injectedDecode;
  wire                IBusSimplePlugin_decodePc_predictionPcLoad_valid;
  wire       [31:0]   IBusSimplePlugin_decodePc_predictionPcLoad_payload;
  reg                 IBusSimplePlugin_iBusRsp_redoFetch;
  wire                IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  reg                 IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire                IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire                _zz_57_;
  wire                _zz_58_;
  wire                IBusSimplePlugin_iBusRsp_flush;
  wire                _zz_59_;
  reg                 _zz_60_;
  reg        [31:0]   _zz_61_;
  reg                 IBusSimplePlugin_iBusRsp_readyForError;
  wire                IBusSimplePlugin_iBusRsp_output_valid;
  wire                IBusSimplePlugin_iBusRsp_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_output_payload_pc;
  wire                IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
  wire                IBusSimplePlugin_iBusRsp_output_payload_isRvc;
  wire                IBusSimplePlugin_decompressor_input_valid;
  reg                 IBusSimplePlugin_decompressor_input_ready;
  wire       [31:0]   IBusSimplePlugin_decompressor_input_payload_pc;
  wire                IBusSimplePlugin_decompressor_input_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_decompressor_input_payload_rsp_inst;
  wire                IBusSimplePlugin_decompressor_input_payload_isRvc;
  wire                IBusSimplePlugin_decompressor_output_valid;
  wire                IBusSimplePlugin_decompressor_output_ready;
  wire       [31:0]   IBusSimplePlugin_decompressor_output_payload_pc;
  wire                IBusSimplePlugin_decompressor_output_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_decompressor_output_payload_rsp_inst;
  wire                IBusSimplePlugin_decompressor_output_payload_isRvc;
  wire                IBusSimplePlugin_decompressor_flushNext;
  wire                IBusSimplePlugin_decompressor_consumeCurrent;
  reg                 IBusSimplePlugin_decompressor_bufferValid;
  reg        [15:0]   IBusSimplePlugin_decompressor_bufferData;
  wire                IBusSimplePlugin_decompressor_isInputLowRvc;
  wire                IBusSimplePlugin_decompressor_isInputHighRvc;
  reg                 IBusSimplePlugin_decompressor_throw2BytesReg;
  wire                IBusSimplePlugin_decompressor_throw2Bytes;
  wire                IBusSimplePlugin_decompressor_unaligned;
  wire       [31:0]   IBusSimplePlugin_decompressor_raw;
  wire                IBusSimplePlugin_decompressor_isRvc;
  wire       [15:0]   _zz_62_;
  reg        [31:0]   IBusSimplePlugin_decompressor_decompressed;
  wire       [4:0]    _zz_63_;
  wire       [4:0]    _zz_64_;
  wire       [11:0]   _zz_65_;
  wire                _zz_66_;
  reg        [11:0]   _zz_67_;
  wire                _zz_68_;
  reg        [9:0]    _zz_69_;
  wire       [20:0]   _zz_70_;
  wire                _zz_71_;
  reg        [14:0]   _zz_72_;
  wire                _zz_73_;
  reg        [2:0]    _zz_74_;
  wire                _zz_75_;
  reg        [9:0]    _zz_76_;
  wire       [20:0]   _zz_77_;
  wire                _zz_78_;
  reg        [4:0]    _zz_79_;
  wire       [12:0]   _zz_80_;
  wire       [4:0]    _zz_81_;
  wire       [4:0]    _zz_82_;
  wire       [4:0]    _zz_83_;
  wire                _zz_84_;
  reg        [2:0]    _zz_85_;
  reg        [2:0]    _zz_86_;
  wire                _zz_87_;
  reg        [6:0]    _zz_88_;
  wire                IBusSimplePlugin_decompressor_bufferFill;
  wire                IBusSimplePlugin_injector_decodeInput_valid;
  wire                IBusSimplePlugin_injector_decodeInput_ready;
  wire       [31:0]   IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire                IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire                IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg                 _zz_89_;
  reg        [31:0]   _zz_90_;
  reg                 _zz_91_;
  reg        [31:0]   _zz_92_;
  reg                 _zz_93_;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_2;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_3;
  reg        [31:0]   IBusSimplePlugin_injector_formal_rawInDecode;
  wire                IBusSimplePlugin_predictor_historyWriteDelayPatched_valid;
  wire       [9:0]    IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_address;
  wire       [19:0]   IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_source;
  wire       [1:0]    IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_branchWish;
  wire                IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_last2Bytes;
  wire       [31:0]   IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_target;
  reg                 IBusSimplePlugin_predictor_historyWrite_valid;
  reg        [9:0]    IBusSimplePlugin_predictor_historyWrite_payload_address;
  wire       [19:0]   IBusSimplePlugin_predictor_historyWrite_payload_data_source;
  reg        [1:0]    IBusSimplePlugin_predictor_historyWrite_payload_data_branchWish;
  wire                IBusSimplePlugin_predictor_historyWrite_payload_data_last2Bytes;
  wire       [31:0]   IBusSimplePlugin_predictor_historyWrite_payload_data_target;
  reg                 IBusSimplePlugin_predictor_writeLast_valid;
  reg        [9:0]    IBusSimplePlugin_predictor_writeLast_payload_address;
  reg        [19:0]   IBusSimplePlugin_predictor_writeLast_payload_data_source;
  reg        [1:0]    IBusSimplePlugin_predictor_writeLast_payload_data_branchWish;
  reg                 IBusSimplePlugin_predictor_writeLast_payload_data_last2Bytes;
  reg        [31:0]   IBusSimplePlugin_predictor_writeLast_payload_data_target;
  wire       [29:0]   _zz_94_;
  wire       [19:0]   IBusSimplePlugin_predictor_buffer_line_source;
  wire       [1:0]    IBusSimplePlugin_predictor_buffer_line_branchWish;
  wire                IBusSimplePlugin_predictor_buffer_line_last2Bytes;
  wire       [31:0]   IBusSimplePlugin_predictor_buffer_line_target;
  wire       [54:0]   _zz_95_;
  reg                 IBusSimplePlugin_predictor_buffer_pcCorrected;
  wire                IBusSimplePlugin_predictor_buffer_hazard;
  reg        [19:0]   IBusSimplePlugin_predictor_line_source;
  reg        [1:0]    IBusSimplePlugin_predictor_line_branchWish;
  reg                 IBusSimplePlugin_predictor_line_last2Bytes;
  reg        [31:0]   IBusSimplePlugin_predictor_line_target;
  reg                 IBusSimplePlugin_predictor_buffer_hazard_regNextWhen;
  wire                IBusSimplePlugin_predictor_hazard;
  reg                 IBusSimplePlugin_predictor_hit;
  wire                IBusSimplePlugin_predictor_fetchContext_hazard;
  wire                IBusSimplePlugin_predictor_fetchContext_hit;
  wire       [19:0]   IBusSimplePlugin_predictor_fetchContext_line_source;
  wire       [1:0]    IBusSimplePlugin_predictor_fetchContext_line_branchWish;
  wire                IBusSimplePlugin_predictor_fetchContext_line_last2Bytes;
  wire       [31:0]   IBusSimplePlugin_predictor_fetchContext_line_target;
  wire                IBusSimplePlugin_predictor_iBusRspContextOutput_hazard;
  reg                 IBusSimplePlugin_predictor_iBusRspContextOutput_hit;
  wire       [19:0]   IBusSimplePlugin_predictor_iBusRspContextOutput_line_source;
  wire       [1:0]    IBusSimplePlugin_predictor_iBusRspContextOutput_line_branchWish;
  wire                IBusSimplePlugin_predictor_iBusRspContextOutput_line_last2Bytes;
  wire       [31:0]   IBusSimplePlugin_predictor_iBusRspContextOutput_line_target;
  reg                 IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_hazard;
  reg                 IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_hit;
  reg        [19:0]   IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_source;
  reg        [1:0]    IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_branchWish;
  reg                 IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_last2Bytes;
  reg        [31:0]   IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_target;
  wire                IBusSimplePlugin_predictor_injectorContext_hazard;
  wire                IBusSimplePlugin_predictor_injectorContext_hit;
  wire       [19:0]   IBusSimplePlugin_predictor_injectorContext_line_source;
  wire       [1:0]    IBusSimplePlugin_predictor_injectorContext_line_branchWish;
  wire                IBusSimplePlugin_predictor_injectorContext_line_last2Bytes;
  wire       [31:0]   IBusSimplePlugin_predictor_injectorContext_line_target;
  wire                IBusSimplePlugin_predictor_compressor_predictionBranch;
  wire                IBusSimplePlugin_predictor_compressor_unalignedWordIssue;
  reg                 IBusSimplePlugin_cmd_valid;
  wire                IBusSimplePlugin_cmd_ready;
  wire       [31:0]   IBusSimplePlugin_cmd_payload_pc;
  wire                IBusSimplePlugin_pending_inc;
  wire                IBusSimplePlugin_pending_dec;
  reg        [2:0]    IBusSimplePlugin_pending_value;
  wire       [2:0]    IBusSimplePlugin_pending_next;
  wire                IBusSimplePlugin_cmdFork_canEmit;
  reg        [31:0]   IBusSimplePlugin_mmu_joinCtx_physicalAddress;
  reg                 IBusSimplePlugin_mmu_joinCtx_isIoAccess;
  reg                 IBusSimplePlugin_mmu_joinCtx_isPaging;
  reg                 IBusSimplePlugin_mmu_joinCtx_allowRead;
  reg                 IBusSimplePlugin_mmu_joinCtx_allowWrite;
  reg                 IBusSimplePlugin_mmu_joinCtx_allowExecute;
  reg                 IBusSimplePlugin_mmu_joinCtx_exception;
  reg                 IBusSimplePlugin_mmu_joinCtx_refilling;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_output_valid;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_output_ready;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst;
  reg        [2:0]    IBusSimplePlugin_rspJoin_rspBuffer_discardCounter;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_flush;
  wire       [31:0]   IBusSimplePlugin_rspJoin_fetchRsp_pc;
  reg                 IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  wire                IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  wire                IBusSimplePlugin_rspJoin_join_valid;
  wire                IBusSimplePlugin_rspJoin_join_ready;
  wire       [31:0]   IBusSimplePlugin_rspJoin_join_payload_pc;
  wire                IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  wire                IBusSimplePlugin_rspJoin_join_payload_isRvc;
  reg                 IBusSimplePlugin_rspJoin_exceptionDetected;
  wire                _zz_96_;
  reg                 dBus_cmd_valid;
  wire                dBus_cmd_ready;
  reg                 dBus_cmd_payload_wr;
  reg        [31:0]   dBus_cmd_payload_address;
  reg        [31:0]   dBus_cmd_payload_data;
  reg        [1:0]    dBus_cmd_payload_size;
  wire                dBus_rsp_ready;
  wire                dBus_rsp_error;
  wire       [31:0]   dBus_rsp_data;
  wire                _zz_97_;
  reg                 execute_DBusSimplePlugin_skipCmd;
  reg        [31:0]   _zz_98_;
  reg        [3:0]    _zz_99_;
  wire       [3:0]    execute_DBusSimplePlugin_formalMask;
  reg                 execute_DBusSimplePlugin_atomic_reserved;
  reg        [31:0]   writeBack_DBusSimplePlugin_rspShifted;
  wire                _zz_100_;
  reg        [31:0]   _zz_101_;
  wire                _zz_102_;
  reg        [31:0]   _zz_103_;
  reg        [31:0]   writeBack_DBusSimplePlugin_rspFormated;
  reg        [1:0]    _zz_104_;
  wire       [32:0]   _zz_105_;
  wire                _zz_106_;
  wire                _zz_107_;
  wire                _zz_108_;
  wire                _zz_109_;
  wire                _zz_110_;
  wire                _zz_111_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_112_;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_113_;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_114_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_115_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_116_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_117_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_118_;
  wire       [4:0]    decode_RegFilePlugin_regFileReadAddress1;
  wire       [4:0]    decode_RegFilePlugin_regFileReadAddress2;
  wire       [31:0]   decode_RegFilePlugin_rs1Data;
  wire       [31:0]   decode_RegFilePlugin_rs2Data;
  reg                 lastStageRegFileWrite_valid /* verilator public */ ;
  wire       [4:0]    lastStageRegFileWrite_payload_address /* verilator public */ ;
  wire       [31:0]   lastStageRegFileWrite_payload_data /* verilator public */ ;
  reg                 _zz_119_;
  reg        [31:0]   execute_IntAluPlugin_bitwise;
  reg        [31:0]   _zz_120_;
  reg        [31:0]   _zz_121_;
  wire                _zz_122_;
  reg        [19:0]   _zz_123_;
  wire                _zz_124_;
  reg        [19:0]   _zz_125_;
  reg        [31:0]   _zz_126_;
  reg        [31:0]   execute_SrcPlugin_addSub;
  wire                execute_SrcPlugin_less;
  wire       [4:0]    execute_FullBarrelShifterPlugin_amplitude;
  reg        [31:0]   _zz_127_;
  wire       [31:0]   execute_FullBarrelShifterPlugin_reversed;
  reg        [31:0]   _zz_128_;
  reg                 _zz_129_;
  reg                 _zz_130_;
  reg                 _zz_131_;
  reg        [4:0]    _zz_132_;
  reg        [31:0]   _zz_133_;
  wire                _zz_134_;
  wire                _zz_135_;
  wire                _zz_136_;
  wire                _zz_137_;
  wire                _zz_138_;
  wire                _zz_139_;
  reg                 execute_MulPlugin_aSigned;
  reg                 execute_MulPlugin_bSigned;
  wire       [31:0]   execute_MulPlugin_a;
  wire       [31:0]   execute_MulPlugin_b;
  wire       [15:0]   execute_MulPlugin_aULow;
  wire       [15:0]   execute_MulPlugin_bULow;
  wire       [16:0]   execute_MulPlugin_aSLow;
  wire       [16:0]   execute_MulPlugin_bSLow;
  wire       [16:0]   execute_MulPlugin_aHigh;
  wire       [16:0]   execute_MulPlugin_bHigh;
  wire       [65:0]   writeBack_MulPlugin_result;
  reg        [32:0]   memory_MulDivIterativePlugin_rs1;
  reg        [31:0]   memory_MulDivIterativePlugin_rs2;
  reg        [64:0]   memory_MulDivIterativePlugin_accumulator;
  wire                memory_MulDivIterativePlugin_frontendOk;
  reg                 memory_MulDivIterativePlugin_div_needRevert;
  reg                 memory_MulDivIterativePlugin_div_counter_willIncrement;
  reg                 memory_MulDivIterativePlugin_div_counter_willClear;
  reg        [5:0]    memory_MulDivIterativePlugin_div_counter_valueNext;
  reg        [5:0]    memory_MulDivIterativePlugin_div_counter_value;
  wire                memory_MulDivIterativePlugin_div_counter_willOverflowIfInc;
  wire                memory_MulDivIterativePlugin_div_counter_willOverflow;
  reg                 memory_MulDivIterativePlugin_div_done;
  reg        [31:0]   memory_MulDivIterativePlugin_div_result;
  wire       [31:0]   _zz_140_;
  wire       [32:0]   memory_MulDivIterativePlugin_div_stage_0_remainderShifted;
  wire       [32:0]   memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator;
  wire       [31:0]   memory_MulDivIterativePlugin_div_stage_0_outRemainder;
  wire       [31:0]   memory_MulDivIterativePlugin_div_stage_0_outNumerator;
  wire       [31:0]   _zz_141_;
  wire                _zz_142_;
  wire                _zz_143_;
  reg        [32:0]   _zz_144_;
  reg        [1:0]    _zz_145_;
  wire       [1:0]    CsrPlugin_misa_base;
  wire       [25:0]   CsrPlugin_misa_extensions;
  reg        [1:0]    CsrPlugin_mtvec_mode;
  reg        [29:0]   CsrPlugin_mtvec_base;
  reg        [31:0]   CsrPlugin_mepc;
  reg                 CsrPlugin_mstatus_MIE;
  reg                 CsrPlugin_mstatus_MPIE;
  reg        [1:0]    CsrPlugin_mstatus_MPP;
  reg                 CsrPlugin_mip_MEIP;
  reg                 CsrPlugin_mip_MTIP;
  reg                 CsrPlugin_mip_MSIP;
  reg                 CsrPlugin_mie_MEIE;
  reg                 CsrPlugin_mie_MTIE;
  reg                 CsrPlugin_mie_MSIE;
  reg        [31:0]   CsrPlugin_mscratch;
  reg                 CsrPlugin_mcause_interrupt;
  reg        [3:0]    CsrPlugin_mcause_exceptionCode;
  reg        [31:0]   CsrPlugin_mtval;
  reg        [63:0]   CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg        [63:0]   CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg                 CsrPlugin_medeleg_IAM;
  reg                 CsrPlugin_medeleg_IAF;
  reg                 CsrPlugin_medeleg_II;
  reg                 CsrPlugin_medeleg_LAM;
  reg                 CsrPlugin_medeleg_LAF;
  reg                 CsrPlugin_medeleg_SAM;
  reg                 CsrPlugin_medeleg_SAF;
  reg                 CsrPlugin_medeleg_EU;
  reg                 CsrPlugin_medeleg_ES;
  reg                 CsrPlugin_medeleg_IPF;
  reg                 CsrPlugin_medeleg_LPF;
  reg                 CsrPlugin_medeleg_SPF;
  reg                 CsrPlugin_mideleg_ST;
  reg                 CsrPlugin_mideleg_SE;
  reg                 CsrPlugin_mideleg_SS;
  reg                 CsrPlugin_sstatus_SIE;
  reg                 CsrPlugin_sstatus_SPIE;
  reg        [0:0]    CsrPlugin_sstatus_SPP;
  reg                 CsrPlugin_sip_SEIP_SOFT;
  reg                 CsrPlugin_sip_SEIP_INPUT;
  wire                CsrPlugin_sip_SEIP_OR;
  reg                 CsrPlugin_sip_STIP;
  reg                 CsrPlugin_sip_SSIP;
  reg                 CsrPlugin_sie_SEIE;
  reg                 CsrPlugin_sie_STIE;
  reg                 CsrPlugin_sie_SSIE;
  reg        [1:0]    CsrPlugin_stvec_mode;
  reg        [29:0]   CsrPlugin_stvec_base;
  reg        [31:0]   CsrPlugin_sscratch;
  reg                 CsrPlugin_scause_interrupt;
  reg        [3:0]    CsrPlugin_scause_exceptionCode;
  reg        [31:0]   CsrPlugin_stval;
  reg        [31:0]   CsrPlugin_sepc;
  reg        [21:0]   CsrPlugin_satp_PPN;
  reg        [8:0]    CsrPlugin_satp_ASID;
  reg        [0:0]    CsrPlugin_satp_MODE;
  wire                _zz_146_;
  wire                _zz_147_;
  wire                _zz_148_;
  wire                _zz_149_;
  wire                _zz_150_;
  wire                _zz_151_;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  reg                 CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  reg        [3:0]    CsrPlugin_exceptionPortCtrl_exceptionContext_code;
  reg        [31:0]   CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
  reg        [1:0]    CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped;
  wire       [1:0]    CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
  wire       [1:0]    _zz_152_;
  wire                _zz_153_;
  reg                 CsrPlugin_interrupt_valid;
  reg        [3:0]    CsrPlugin_interrupt_code /* verilator public */ ;
  reg        [1:0]    CsrPlugin_interrupt_targetPrivilege;
  wire                CsrPlugin_exception;
  reg                 CsrPlugin_lastStageWasWfi;
  reg                 CsrPlugin_pipelineLiberator_pcValids_0;
  reg                 CsrPlugin_pipelineLiberator_pcValids_1;
  reg                 CsrPlugin_pipelineLiberator_pcValids_2;
  wire                CsrPlugin_pipelineLiberator_active;
  reg                 CsrPlugin_pipelineLiberator_done;
  wire                CsrPlugin_interruptJump /* verilator public */ ;
  reg                 CsrPlugin_hadException;
  reg        [1:0]    CsrPlugin_targetPrivilege;
  reg        [3:0]    CsrPlugin_trapCause;
  reg        [1:0]    CsrPlugin_xtvec_mode;
  reg        [29:0]   CsrPlugin_xtvec_base;
  reg                 execute_CsrPlugin_wfiWake;
  wire                execute_CsrPlugin_blockedBySideEffects;
  reg                 execute_CsrPlugin_illegalAccess;
  reg                 execute_CsrPlugin_illegalInstruction;
  wire       [31:0]   execute_CsrPlugin_readData;
  reg                 execute_CsrPlugin_writeInstruction;
  reg                 execute_CsrPlugin_readInstruction;
  wire                execute_CsrPlugin_writeEnable;
  wire                execute_CsrPlugin_readEnable;
  reg        [31:0]   execute_CsrPlugin_readToWriteData;
  reg        [31:0]   execute_CsrPlugin_writeData;
  wire       [11:0]   execute_CsrPlugin_csrAddress;
  reg                 DebugPlugin_firstCycle;
  reg                 DebugPlugin_secondCycle;
  reg                 DebugPlugin_resetIt;
  reg                 DebugPlugin_haltIt;
  reg                 DebugPlugin_stepIt;
  reg                 DebugPlugin_isPipBusy;
  reg                 DebugPlugin_godmode;
  reg                 DebugPlugin_haltedByBreak;
  reg        [31:0]   DebugPlugin_busReadDataReg;
  reg                 _zz_154_;
  wire                DebugPlugin_allowEBreak;
  reg                 _zz_155_;
  reg                 DebugPlugin_resetIt_regNext;
  wire                execute_BranchPlugin_eq;
  wire       [2:0]    _zz_156_;
  reg                 _zz_157_;
  reg                 _zz_158_;
  wire       [31:0]   execute_BranchPlugin_branch_src1;
  wire                _zz_159_;
  reg        [10:0]   _zz_160_;
  wire                _zz_161_;
  reg        [19:0]   _zz_162_;
  wire                _zz_163_;
  reg        [18:0]   _zz_164_;
  reg        [31:0]   _zz_165_;
  wire       [31:0]   execute_BranchPlugin_branchAdder;
  wire                memory_BranchPlugin_predictionMissmatch;
  reg                 MmuPlugin_status_sum;
  reg                 MmuPlugin_status_mxr;
  reg                 MmuPlugin_status_mprv;
  reg                 MmuPlugin_satp_mode;
  reg        [19:0]   MmuPlugin_satp_ppn;
  reg                 MmuPlugin_ports_0_cache_0_valid;
  reg                 MmuPlugin_ports_0_cache_0_exception;
  reg                 MmuPlugin_ports_0_cache_0_superPage;
  reg        [9:0]    MmuPlugin_ports_0_cache_0_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_0_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_0_cache_0_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_0_physicalAddress_1;
  reg                 MmuPlugin_ports_0_cache_0_allowRead;
  reg                 MmuPlugin_ports_0_cache_0_allowWrite;
  reg                 MmuPlugin_ports_0_cache_0_allowExecute;
  reg                 MmuPlugin_ports_0_cache_0_allowUser;
  reg                 MmuPlugin_ports_0_cache_1_valid;
  reg                 MmuPlugin_ports_0_cache_1_exception;
  reg                 MmuPlugin_ports_0_cache_1_superPage;
  reg        [9:0]    MmuPlugin_ports_0_cache_1_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_1_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_0_cache_1_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_1_physicalAddress_1;
  reg                 MmuPlugin_ports_0_cache_1_allowRead;
  reg                 MmuPlugin_ports_0_cache_1_allowWrite;
  reg                 MmuPlugin_ports_0_cache_1_allowExecute;
  reg                 MmuPlugin_ports_0_cache_1_allowUser;
  reg                 MmuPlugin_ports_0_cache_2_valid;
  reg                 MmuPlugin_ports_0_cache_2_exception;
  reg                 MmuPlugin_ports_0_cache_2_superPage;
  reg        [9:0]    MmuPlugin_ports_0_cache_2_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_2_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_0_cache_2_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_2_physicalAddress_1;
  reg                 MmuPlugin_ports_0_cache_2_allowRead;
  reg                 MmuPlugin_ports_0_cache_2_allowWrite;
  reg                 MmuPlugin_ports_0_cache_2_allowExecute;
  reg                 MmuPlugin_ports_0_cache_2_allowUser;
  reg                 MmuPlugin_ports_0_cache_3_valid;
  reg                 MmuPlugin_ports_0_cache_3_exception;
  reg                 MmuPlugin_ports_0_cache_3_superPage;
  reg        [9:0]    MmuPlugin_ports_0_cache_3_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_3_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_0_cache_3_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_0_cache_3_physicalAddress_1;
  reg                 MmuPlugin_ports_0_cache_3_allowRead;
  reg                 MmuPlugin_ports_0_cache_3_allowWrite;
  reg                 MmuPlugin_ports_0_cache_3_allowExecute;
  reg                 MmuPlugin_ports_0_cache_3_allowUser;
  wire                MmuPlugin_ports_0_cacheHits_0;
  wire                MmuPlugin_ports_0_cacheHits_1;
  wire                MmuPlugin_ports_0_cacheHits_2;
  wire                MmuPlugin_ports_0_cacheHits_3;
  wire                MmuPlugin_ports_0_cacheHit;
  wire                _zz_166_;
  wire                _zz_167_;
  wire       [1:0]    _zz_168_;
  wire                MmuPlugin_ports_0_cacheLine_valid;
  wire                MmuPlugin_ports_0_cacheLine_exception;
  wire                MmuPlugin_ports_0_cacheLine_superPage;
  wire       [9:0]    MmuPlugin_ports_0_cacheLine_virtualAddress_0;
  wire       [9:0]    MmuPlugin_ports_0_cacheLine_virtualAddress_1;
  wire       [9:0]    MmuPlugin_ports_0_cacheLine_physicalAddress_0;
  wire       [9:0]    MmuPlugin_ports_0_cacheLine_physicalAddress_1;
  wire                MmuPlugin_ports_0_cacheLine_allowRead;
  wire                MmuPlugin_ports_0_cacheLine_allowWrite;
  wire                MmuPlugin_ports_0_cacheLine_allowExecute;
  wire                MmuPlugin_ports_0_cacheLine_allowUser;
  reg                 MmuPlugin_ports_0_entryToReplace_willIncrement;
  wire                MmuPlugin_ports_0_entryToReplace_willClear;
  reg        [1:0]    MmuPlugin_ports_0_entryToReplace_valueNext;
  reg        [1:0]    MmuPlugin_ports_0_entryToReplace_value;
  wire                MmuPlugin_ports_0_entryToReplace_willOverflowIfInc;
  wire                MmuPlugin_ports_0_entryToReplace_willOverflow;
  reg                 MmuPlugin_ports_0_requireMmuLockup;
  reg                 MmuPlugin_ports_1_cache_0_valid;
  reg                 MmuPlugin_ports_1_cache_0_exception;
  reg                 MmuPlugin_ports_1_cache_0_superPage;
  reg        [9:0]    MmuPlugin_ports_1_cache_0_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_0_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_1_cache_0_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_0_physicalAddress_1;
  reg                 MmuPlugin_ports_1_cache_0_allowRead;
  reg                 MmuPlugin_ports_1_cache_0_allowWrite;
  reg                 MmuPlugin_ports_1_cache_0_allowExecute;
  reg                 MmuPlugin_ports_1_cache_0_allowUser;
  reg                 MmuPlugin_ports_1_cache_1_valid;
  reg                 MmuPlugin_ports_1_cache_1_exception;
  reg                 MmuPlugin_ports_1_cache_1_superPage;
  reg        [9:0]    MmuPlugin_ports_1_cache_1_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_1_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_1_cache_1_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_1_physicalAddress_1;
  reg                 MmuPlugin_ports_1_cache_1_allowRead;
  reg                 MmuPlugin_ports_1_cache_1_allowWrite;
  reg                 MmuPlugin_ports_1_cache_1_allowExecute;
  reg                 MmuPlugin_ports_1_cache_1_allowUser;
  reg                 MmuPlugin_ports_1_cache_2_valid;
  reg                 MmuPlugin_ports_1_cache_2_exception;
  reg                 MmuPlugin_ports_1_cache_2_superPage;
  reg        [9:0]    MmuPlugin_ports_1_cache_2_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_2_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_1_cache_2_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_2_physicalAddress_1;
  reg                 MmuPlugin_ports_1_cache_2_allowRead;
  reg                 MmuPlugin_ports_1_cache_2_allowWrite;
  reg                 MmuPlugin_ports_1_cache_2_allowExecute;
  reg                 MmuPlugin_ports_1_cache_2_allowUser;
  reg                 MmuPlugin_ports_1_cache_3_valid;
  reg                 MmuPlugin_ports_1_cache_3_exception;
  reg                 MmuPlugin_ports_1_cache_3_superPage;
  reg        [9:0]    MmuPlugin_ports_1_cache_3_virtualAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_3_virtualAddress_1;
  reg        [9:0]    MmuPlugin_ports_1_cache_3_physicalAddress_0;
  reg        [9:0]    MmuPlugin_ports_1_cache_3_physicalAddress_1;
  reg                 MmuPlugin_ports_1_cache_3_allowRead;
  reg                 MmuPlugin_ports_1_cache_3_allowWrite;
  reg                 MmuPlugin_ports_1_cache_3_allowExecute;
  reg                 MmuPlugin_ports_1_cache_3_allowUser;
  wire                MmuPlugin_ports_1_cacheHits_0;
  wire                MmuPlugin_ports_1_cacheHits_1;
  wire                MmuPlugin_ports_1_cacheHits_2;
  wire                MmuPlugin_ports_1_cacheHits_3;
  wire                MmuPlugin_ports_1_cacheHit;
  wire                _zz_169_;
  wire                _zz_170_;
  wire       [1:0]    _zz_171_;
  wire                MmuPlugin_ports_1_cacheLine_valid;
  wire                MmuPlugin_ports_1_cacheLine_exception;
  wire                MmuPlugin_ports_1_cacheLine_superPage;
  wire       [9:0]    MmuPlugin_ports_1_cacheLine_virtualAddress_0;
  wire       [9:0]    MmuPlugin_ports_1_cacheLine_virtualAddress_1;
  wire       [9:0]    MmuPlugin_ports_1_cacheLine_physicalAddress_0;
  wire       [9:0]    MmuPlugin_ports_1_cacheLine_physicalAddress_1;
  wire                MmuPlugin_ports_1_cacheLine_allowRead;
  wire                MmuPlugin_ports_1_cacheLine_allowWrite;
  wire                MmuPlugin_ports_1_cacheLine_allowExecute;
  wire                MmuPlugin_ports_1_cacheLine_allowUser;
  reg                 MmuPlugin_ports_1_entryToReplace_willIncrement;
  wire                MmuPlugin_ports_1_entryToReplace_willClear;
  reg        [1:0]    MmuPlugin_ports_1_entryToReplace_valueNext;
  reg        [1:0]    MmuPlugin_ports_1_entryToReplace_value;
  wire                MmuPlugin_ports_1_entryToReplace_willOverflowIfInc;
  wire                MmuPlugin_ports_1_entryToReplace_willOverflow;
  reg                 MmuPlugin_ports_1_requireMmuLockup;
  reg        `MmuPlugin_shared_State_defaultEncoding_type MmuPlugin_shared_state_1_;
  reg        [9:0]    MmuPlugin_shared_vpn_0;
  reg        [9:0]    MmuPlugin_shared_vpn_1;
  reg        [0:0]    MmuPlugin_shared_portId;
  wire                MmuPlugin_shared_dBusRsp_pte_V;
  wire                MmuPlugin_shared_dBusRsp_pte_R;
  wire                MmuPlugin_shared_dBusRsp_pte_W;
  wire                MmuPlugin_shared_dBusRsp_pte_X;
  wire                MmuPlugin_shared_dBusRsp_pte_U;
  wire                MmuPlugin_shared_dBusRsp_pte_G;
  wire                MmuPlugin_shared_dBusRsp_pte_A;
  wire                MmuPlugin_shared_dBusRsp_pte_D;
  wire       [1:0]    MmuPlugin_shared_dBusRsp_pte_RSW;
  wire       [9:0]    MmuPlugin_shared_dBusRsp_pte_PPN0;
  wire       [11:0]   MmuPlugin_shared_dBusRsp_pte_PPN1;
  wire                MmuPlugin_shared_dBusRsp_exception;
  wire                MmuPlugin_shared_dBusRsp_leaf;
  reg                 MmuPlugin_shared_pteBuffer_V;
  reg                 MmuPlugin_shared_pteBuffer_R;
  reg                 MmuPlugin_shared_pteBuffer_W;
  reg                 MmuPlugin_shared_pteBuffer_X;
  reg                 MmuPlugin_shared_pteBuffer_U;
  reg                 MmuPlugin_shared_pteBuffer_G;
  reg                 MmuPlugin_shared_pteBuffer_A;
  reg                 MmuPlugin_shared_pteBuffer_D;
  reg        [1:0]    MmuPlugin_shared_pteBuffer_RSW;
  reg        [9:0]    MmuPlugin_shared_pteBuffer_PPN0;
  reg        [11:0]   MmuPlugin_shared_pteBuffer_PPN1;
  reg        [31:0]   memory_to_writeBack_MEMORY_READ_DATA;
  reg        [31:0]   decode_to_execute_SRC1;
  reg        [31:0]   execute_to_memory_MUL_LL;
  reg                 decode_to_execute_IS_DIV;
  reg                 execute_to_memory_IS_DIV;
  reg        [31:0]   decode_to_execute_SRC2;
  reg                 decode_to_execute_SRC_USE_SUB_LESS;
  reg        [31:0]   execute_to_memory_MMU_RSP_physicalAddress;
  reg                 execute_to_memory_MMU_RSP_isIoAccess;
  reg                 execute_to_memory_MMU_RSP_isPaging;
  reg                 execute_to_memory_MMU_RSP_allowRead;
  reg                 execute_to_memory_MMU_RSP_allowWrite;
  reg                 execute_to_memory_MMU_RSP_allowExecute;
  reg                 execute_to_memory_MMU_RSP_exception;
  reg                 execute_to_memory_MMU_RSP_refilling;
  reg                 execute_to_memory_TARGET_MISSMATCH2;
  reg                 execute_to_memory_BRANCH_DO;
  reg        [31:0]   decode_to_execute_RS2;
  reg                 decode_to_execute_CSR_READ_OPCODE;
  reg                 decode_to_execute_IS_RS2_SIGNED;
  reg                 decode_to_execute_IS_RS1_SIGNED;
  reg        [1:0]    execute_to_memory_MEMORY_ADDRESS_LOW;
  reg        [1:0]    memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg        [33:0]   execute_to_memory_MUL_HL;
  reg                 execute_to_memory_ALIGNEMENT_FAULT;
  reg        [31:0]   execute_to_memory_SHIFT_RIGHT;
  reg        [31:0]   execute_to_memory_NEXT_PC2;
  reg        [31:0]   execute_to_memory_BRANCH_CALC;
  reg        [31:0]   decode_to_execute_PC;
  reg        [31:0]   execute_to_memory_PC;
  reg        [31:0]   memory_to_writeBack_PC;
  reg                 decode_to_execute_CSR_WRITE_OPCODE;
  reg        [51:0]   memory_to_writeBack_MUL_LOW;
  reg        `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg        `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg        `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg                 decode_to_execute_MEMORY_ATOMIC;
  reg                 execute_to_memory_MEMORY_ATOMIC;
  reg                 memory_to_writeBack_MEMORY_ATOMIC;
  reg        [31:0]   decode_to_execute_RS1;
  reg                 decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg                 execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg                 decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg        `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg                 decode_to_execute_REGFILE_WRITE_VALID;
  reg                 execute_to_memory_REGFILE_WRITE_VALID;
  reg                 memory_to_writeBack_REGFILE_WRITE_VALID;
  reg                 decode_to_execute_PREDICTION_CONTEXT_hazard;
  reg                 decode_to_execute_PREDICTION_CONTEXT_hit;
  reg        [19:0]   decode_to_execute_PREDICTION_CONTEXT_line_source;
  reg        [1:0]    decode_to_execute_PREDICTION_CONTEXT_line_branchWish;
  reg                 decode_to_execute_PREDICTION_CONTEXT_line_last2Bytes;
  reg        [31:0]   decode_to_execute_PREDICTION_CONTEXT_line_target;
  reg                 execute_to_memory_PREDICTION_CONTEXT_hazard;
  reg                 execute_to_memory_PREDICTION_CONTEXT_hit;
  reg        [19:0]   execute_to_memory_PREDICTION_CONTEXT_line_source;
  reg        [1:0]    execute_to_memory_PREDICTION_CONTEXT_line_branchWish;
  reg                 execute_to_memory_PREDICTION_CONTEXT_line_last2Bytes;
  reg        [31:0]   execute_to_memory_PREDICTION_CONTEXT_line_target;
  reg                 decode_to_execute_MEMORY_ENABLE;
  reg                 execute_to_memory_MEMORY_ENABLE;
  reg                 memory_to_writeBack_MEMORY_ENABLE;
  reg                 decode_to_execute_IS_CSR;
  reg                 decode_to_execute_IS_MUL;
  reg                 execute_to_memory_IS_MUL;
  reg                 memory_to_writeBack_IS_MUL;
  reg        [33:0]   execute_to_memory_MUL_HH;
  reg        [33:0]   memory_to_writeBack_MUL_HH;
  reg        [31:0]   decode_to_execute_INSTRUCTION;
  reg        [31:0]   execute_to_memory_INSTRUCTION;
  reg        [31:0]   memory_to_writeBack_INSTRUCTION;
  reg                 decode_to_execute_MEMORY_STORE;
  reg                 execute_to_memory_MEMORY_STORE;
  reg                 memory_to_writeBack_MEMORY_STORE;
  reg                 decode_to_execute_SRC_LESS_UNSIGNED;
  reg        [33:0]   execute_to_memory_MUL_LH;
  reg                 decode_to_execute_IS_RVC;
  reg                 execute_to_memory_IS_RVC;
  reg                 execute_to_memory_MMU_FAULT;
  reg                 decode_to_execute_IS_SFENCE_VMA;
  reg                 execute_to_memory_IS_SFENCE_VMA;
  reg                 memory_to_writeBack_IS_SFENCE_VMA;
  reg        `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg        [31:0]   decode_to_execute_FORMAL_PC_NEXT;
  reg        [31:0]   execute_to_memory_FORMAL_PC_NEXT;
  reg        [31:0]   memory_to_writeBack_FORMAL_PC_NEXT;
  reg        [31:0]   execute_to_memory_REGFILE_WRITE_DATA;
  reg        [31:0]   memory_to_writeBack_REGFILE_WRITE_DATA;
  reg        `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg        `ShiftCtrlEnum_defaultEncoding_type execute_to_memory_SHIFT_CTRL;
  reg        `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg                 execute_to_memory_ATOMIC_HIT;
  reg                 memory_to_writeBack_ATOMIC_HIT;
  reg                 decode_to_execute_DO_EBREAK;
  reg                 decode_to_execute_SRC2_FORCE_ZERO;
  reg        [2:0]    _zz_172_;
  reg                 execute_CsrPlugin_csr_3857;
  reg                 execute_CsrPlugin_csr_3858;
  reg                 execute_CsrPlugin_csr_3859;
  reg                 execute_CsrPlugin_csr_3860;
  reg                 execute_CsrPlugin_csr_768;
  reg                 execute_CsrPlugin_csr_836;
  reg                 execute_CsrPlugin_csr_772;
  reg                 execute_CsrPlugin_csr_773;
  reg                 execute_CsrPlugin_csr_833;
  reg                 execute_CsrPlugin_csr_832;
  reg                 execute_CsrPlugin_csr_834;
  reg                 execute_CsrPlugin_csr_835;
  reg                 execute_CsrPlugin_csr_770;
  reg                 execute_CsrPlugin_csr_771;
  reg                 execute_CsrPlugin_csr_256;
  reg                 execute_CsrPlugin_csr_324;
  reg                 execute_CsrPlugin_csr_260;
  reg                 execute_CsrPlugin_csr_261;
  reg                 execute_CsrPlugin_csr_321;
  reg                 execute_CsrPlugin_csr_320;
  reg                 execute_CsrPlugin_csr_322;
  reg                 execute_CsrPlugin_csr_323;
  reg                 execute_CsrPlugin_csr_384;
  reg        [31:0]   _zz_173_;
  reg        [31:0]   _zz_174_;
  reg        [31:0]   _zz_175_;
  reg        [31:0]   _zz_176_;
  reg        [31:0]   _zz_177_;
  reg        [31:0]   _zz_178_;
  reg        [31:0]   _zz_179_;
  reg        [31:0]   _zz_180_;
  reg        [31:0]   _zz_181_;
  reg        [31:0]   _zz_182_;
  reg        [31:0]   _zz_183_;
  reg        [31:0]   _zz_184_;
  reg        [31:0]   _zz_185_;
  reg        [31:0]   _zz_186_;
  reg        [31:0]   _zz_187_;
  reg        [31:0]   _zz_188_;
  reg        [31:0]   _zz_189_;
  reg        [31:0]   _zz_190_;
  reg        [31:0]   _zz_191_;
  wire                iBus_cmd_m2sPipe_valid;
  wire                iBus_cmd_m2sPipe_ready;
  wire       [31:0]   iBus_cmd_m2sPipe_payload_pc;
  reg                 iBus_cmd_m2sPipe_rValid;
  reg        [31:0]   iBus_cmd_m2sPipe_rData_pc;
  wire                dBus_cmd_halfPipe_valid;
  wire                dBus_cmd_halfPipe_ready;
  wire                dBus_cmd_halfPipe_payload_wr;
  wire       [31:0]   dBus_cmd_halfPipe_payload_address;
  wire       [31:0]   dBus_cmd_halfPipe_payload_data;
  wire       [1:0]    dBus_cmd_halfPipe_payload_size;
  reg                 dBus_cmd_halfPipe_regs_valid;
  reg                 dBus_cmd_halfPipe_regs_ready;
  reg                 dBus_cmd_halfPipe_regs_payload_wr;
  reg        [31:0]   dBus_cmd_halfPipe_regs_payload_address;
  reg        [31:0]   dBus_cmd_halfPipe_regs_payload_data;
  reg        [1:0]    dBus_cmd_halfPipe_regs_payload_size;
  reg        [3:0]    _zz_192_;
  reg                 _zz_193_;
  `ifndef SYNTHESIS
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_1__string;
  reg [31:0] _zz_2__string;
  reg [31:0] _zz_3__string;
  reg [71:0] _zz_4__string;
  reg [71:0] _zz_5__string;
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_6__string;
  reg [71:0] _zz_7__string;
  reg [71:0] _zz_8__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_9__string;
  reg [63:0] _zz_10__string;
  reg [63:0] _zz_11__string;
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_12__string;
  reg [39:0] _zz_13__string;
  reg [39:0] _zz_14__string;
  reg [39:0] _zz_15__string;
  reg [39:0] _zz_16__string;
  reg [39:0] _zz_17__string;
  reg [39:0] _zz_18__string;
  reg [39:0] decode_ENV_CTRL_string;
  reg [39:0] _zz_19__string;
  reg [39:0] _zz_20__string;
  reg [39:0] _zz_21__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_22__string;
  reg [39:0] memory_ENV_CTRL_string;
  reg [39:0] _zz_23__string;
  reg [39:0] execute_ENV_CTRL_string;
  reg [39:0] _zz_24__string;
  reg [39:0] writeBack_ENV_CTRL_string;
  reg [39:0] _zz_25__string;
  reg [71:0] memory_SHIFT_CTRL_string;
  reg [71:0] _zz_28__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_29__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_32__string;
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_34__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_35__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_36__string;
  reg [39:0] _zz_40__string;
  reg [71:0] _zz_41__string;
  reg [63:0] _zz_42__string;
  reg [31:0] _zz_43__string;
  reg [95:0] _zz_44__string;
  reg [23:0] _zz_45__string;
  reg [39:0] _zz_46__string;
  reg [39:0] _zz_112__string;
  reg [23:0] _zz_113__string;
  reg [95:0] _zz_114__string;
  reg [31:0] _zz_115__string;
  reg [63:0] _zz_116__string;
  reg [71:0] _zz_117__string;
  reg [39:0] _zz_118__string;
  reg [47:0] MmuPlugin_shared_state_1__string;
  reg [39:0] decode_to_execute_ENV_CTRL_string;
  reg [39:0] execute_to_memory_ENV_CTRL_string;
  reg [39:0] memory_to_writeBack_ENV_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  reg [71:0] execute_to_memory_SHIFT_CTRL_string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  `endif

  reg [54:0] IBusSimplePlugin_predictor_history [0:1023];
  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;

  assign _zz_222_ = (execute_arbitration_isValid && execute_IS_CSR);
  assign _zz_223_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_224_ = 1'b1;
  assign _zz_225_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_226_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_227_ = (memory_arbitration_isValid && memory_IS_DIV);
  assign _zz_228_ = ({decodeExceptionPort_valid,IBusSimplePlugin_decodeExceptionPort_valid} != (2'b00));
  assign _zz_229_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_WFI));
  assign _zz_230_ = (execute_arbitration_isValid && execute_DO_EBREAK);
  assign _zz_231_ = (({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00)) == 1'b0);
  assign _zz_232_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_233_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_234_ = (DebugPlugin_stepIt && IBusSimplePlugin_incomingInstruction);
  assign _zz_235_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_236_ = (IBusSimplePlugin_jump_pcLoad_valid && ((! decode_arbitration_isStuck) || decode_arbitration_removeIt));
  assign _zz_237_ = (((IBusSimplePlugin_predictor_fetchContext_line_branchWish[1] && IBusSimplePlugin_predictor_iBusRspContextOutput_hit) && (! IBusSimplePlugin_predictor_fetchContext_hazard)) && (IBusSimplePlugin_decompressor_output_valid && IBusSimplePlugin_decompressor_output_ready));
  assign _zz_238_ = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_payload_rsp_error);
  assign _zz_239_ = ((IBusSimplePlugin_iBusRsp_stages_1_input_valid && (! IBusSimplePlugin_mmu_joinCtx_refilling)) && (IBusSimplePlugin_mmu_joinCtx_exception || (! IBusSimplePlugin_mmu_joinCtx_allowExecute)));
  assign _zz_240_ = ((dBus_rsp_ready && dBus_rsp_error) && (! memory_MEMORY_STORE));
  assign _zz_241_ = (! ((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (1'b1 || (! memory_arbitration_isStuckByOthers))));
  assign _zz_242_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_243_ = (1'b0 || (! 1'b1));
  assign _zz_244_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_245_ = (1'b0 || (! memory_BYPASSABLE_MEMORY_STAGE));
  assign _zz_246_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_247_ = (1'b0 || (! execute_BYPASSABLE_EXECUTE_STAGE));
  assign _zz_248_ = execute_INSTRUCTION[13 : 12];
  assign _zz_249_ = (memory_MulDivIterativePlugin_frontendOk && (! memory_MulDivIterativePlugin_div_done));
  assign _zz_250_ = (! memory_arbitration_isStuck);
  assign _zz_251_ = (CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]);
  assign _zz_252_ = (execute_CsrPlugin_illegalAccess || execute_CsrPlugin_illegalInstruction);
  assign _zz_253_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_ECALL));
  assign _zz_254_ = debug_bus_cmd_payload_address[7 : 2];
  assign _zz_255_ = ((MmuPlugin_dBusAccess_rsp_valid && (! MmuPlugin_dBusAccess_rsp_payload_redo)) && (MmuPlugin_shared_dBusRsp_leaf || MmuPlugin_shared_dBusRsp_exception));
  assign _zz_256_ = (MmuPlugin_shared_portId == (1'b1));
  assign _zz_257_ = (MmuPlugin_shared_portId == (1'b0));
  assign _zz_258_ = (IBusSimplePlugin_decompressor_output_ready && IBusSimplePlugin_decompressor_input_valid);
  assign _zz_259_ = ((CsrPlugin_sstatus_SIE && (CsrPlugin_privilege == (2'b01))) || (CsrPlugin_privilege < (2'b01)));
  assign _zz_260_ = ((_zz_146_ && (1'b1 && CsrPlugin_mideleg_ST)) && (! 1'b0));
  assign _zz_261_ = ((_zz_147_ && (1'b1 && CsrPlugin_mideleg_SS)) && (! 1'b0));
  assign _zz_262_ = ((_zz_148_ && (1'b1 && CsrPlugin_mideleg_SE)) && (! 1'b0));
  assign _zz_263_ = (CsrPlugin_mstatus_MIE || (CsrPlugin_privilege < (2'b11)));
  assign _zz_264_ = ((_zz_146_ && 1'b1) && (! (CsrPlugin_mideleg_ST != (1'b0))));
  assign _zz_265_ = ((_zz_147_ && 1'b1) && (! (CsrPlugin_mideleg_SS != (1'b0))));
  assign _zz_266_ = ((_zz_148_ && 1'b1) && (! (CsrPlugin_mideleg_SE != (1'b0))));
  assign _zz_267_ = ((_zz_149_ && 1'b1) && (! 1'b0));
  assign _zz_268_ = ((_zz_150_ && 1'b1) && (! 1'b0));
  assign _zz_269_ = ((_zz_151_ && 1'b1) && (! 1'b0));
  assign _zz_270_ = (IBusSimplePlugin_mmuBus_cmd_isValid && IBusSimplePlugin_mmuBus_rsp_refilling);
  assign _zz_271_ = (DBusSimplePlugin_mmuBus_cmd_isValid && DBusSimplePlugin_mmuBus_rsp_refilling);
  assign _zz_272_ = (MmuPlugin_ports_0_entryToReplace_value == (2'b00));
  assign _zz_273_ = (MmuPlugin_ports_0_entryToReplace_value == (2'b01));
  assign _zz_274_ = (MmuPlugin_ports_0_entryToReplace_value == (2'b10));
  assign _zz_275_ = (MmuPlugin_ports_0_entryToReplace_value == (2'b11));
  assign _zz_276_ = (MmuPlugin_ports_1_entryToReplace_value == (2'b00));
  assign _zz_277_ = (MmuPlugin_ports_1_entryToReplace_value == (2'b01));
  assign _zz_278_ = (MmuPlugin_ports_1_entryToReplace_value == (2'b10));
  assign _zz_279_ = (MmuPlugin_ports_1_entryToReplace_value == (2'b11));
  assign _zz_280_ = (! dBus_cmd_halfPipe_regs_valid);
  assign _zz_281_ = {_zz_62_[1 : 0],_zz_62_[15 : 13]};
  assign _zz_282_ = _zz_62_[6 : 5];
  assign _zz_283_ = _zz_62_[11 : 10];
  assign _zz_284_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_285_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_286_ = execute_INSTRUCTION[13];
  assign _zz_287_ = _zz_105_[0 : 0];
  assign _zz_288_ = _zz_105_[1 : 1];
  assign _zz_289_ = _zz_105_[12 : 12];
  assign _zz_290_ = _zz_105_[11 : 11];
  assign _zz_291_ = _zz_105_[13 : 13];
  assign _zz_292_ = _zz_105_[32 : 32];
  assign _zz_293_ = _zz_105_[26 : 26];
  assign _zz_294_ = _zz_105_[2 : 2];
  assign _zz_295_ = ($signed(_zz_296_) + $signed(_zz_301_));
  assign _zz_296_ = ($signed(_zz_297_) + $signed(_zz_299_));
  assign _zz_297_ = 52'h0;
  assign _zz_298_ = {1'b0,memory_MUL_LL};
  assign _zz_299_ = {{19{_zz_298_[32]}}, _zz_298_};
  assign _zz_300_ = ({16'd0,memory_MUL_LH} <<< 16);
  assign _zz_301_ = {{2{_zz_300_[49]}}, _zz_300_};
  assign _zz_302_ = ({16'd0,memory_MUL_HL} <<< 16);
  assign _zz_303_ = {{2{_zz_302_[49]}}, _zz_302_};
  assign _zz_304_ = (execute_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_305_ = {29'd0, _zz_304_};
  assign _zz_306_ = ($signed(_zz_308_) >>> execute_FullBarrelShifterPlugin_amplitude);
  assign _zz_307_ = _zz_306_[31 : 0];
  assign _zz_308_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_FullBarrelShifterPlugin_reversed[31]),execute_FullBarrelShifterPlugin_reversed};
  assign _zz_309_ = _zz_105_[7 : 7];
  assign _zz_310_ = _zz_105_[27 : 27];
  assign _zz_311_ = _zz_105_[17 : 17];
  assign _zz_312_ = _zz_105_[18 : 18];
  assign _zz_313_ = _zz_105_[29 : 29];
  assign _zz_314_ = _zz_105_[8 : 8];
  assign _zz_315_ = _zz_105_[28 : 28];
  assign _zz_316_ = _zz_105_[14 : 14];
  assign _zz_317_ = _zz_105_[23 : 23];
  assign _zz_318_ = _zz_105_[20 : 20];
  assign _zz_319_ = (_zz_52_ - (4'b0001));
  assign _zz_320_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_321_ = {29'd0, _zz_320_};
  assign _zz_322_ = (decode_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_323_ = {29'd0, _zz_322_};
  assign _zz_324_ = {{_zz_72_,_zz_62_[6 : 2]},12'h0};
  assign _zz_325_ = {{{(4'b0000),_zz_62_[8 : 7]},_zz_62_[12 : 9]},(2'b00)};
  assign _zz_326_ = {{{(4'b0000),_zz_62_[8 : 7]},_zz_62_[12 : 9]},(2'b00)};
  assign _zz_327_ = (decode_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_328_ = {29'd0, _zz_327_};
  assign _zz_329_ = _zz_94_[9:0];
  assign _zz_330_ = _zz_95_[22 : 22];
  assign _zz_331_ = (IBusSimplePlugin_iBusRsp_stages_1_input_payload >>> 2);
  assign _zz_332_ = _zz_331_[9:0];
  assign _zz_333_ = (IBusSimplePlugin_iBusRsp_stages_1_input_payload >>> 12);
  assign _zz_334_ = (memory_PREDICTION_CONTEXT_line_branchWish + _zz_336_);
  assign _zz_335_ = (memory_PREDICTION_CONTEXT_line_branchWish == (2'b10));
  assign _zz_336_ = {1'd0, _zz_335_};
  assign _zz_337_ = (memory_PREDICTION_CONTEXT_line_branchWish == (2'b01));
  assign _zz_338_ = {1'd0, _zz_337_};
  assign _zz_339_ = (memory_PREDICTION_CONTEXT_line_branchWish - _zz_341_);
  assign _zz_340_ = memory_PREDICTION_CONTEXT_line_branchWish[1];
  assign _zz_341_ = {1'd0, _zz_340_};
  assign _zz_342_ = (! memory_PREDICTION_CONTEXT_line_branchWish[1]);
  assign _zz_343_ = {1'd0, _zz_342_};
  assign _zz_344_ = (IBusSimplePlugin_iBusRsp_stages_1_input_payload >>> 2);
  assign _zz_345_ = (IBusSimplePlugin_pending_value + _zz_347_);
  assign _zz_346_ = IBusSimplePlugin_pending_inc;
  assign _zz_347_ = {2'd0, _zz_346_};
  assign _zz_348_ = IBusSimplePlugin_pending_dec;
  assign _zz_349_ = {2'd0, _zz_348_};
  assign _zz_350_ = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter != (3'b000)));
  assign _zz_351_ = {2'd0, _zz_350_};
  assign _zz_352_ = IBusSimplePlugin_pending_dec;
  assign _zz_353_ = {2'd0, _zz_352_};
  assign _zz_354_ = (memory_MEMORY_STORE ? (3'b110) : (3'b100));
  assign _zz_355_ = (! writeBack_ATOMIC_HIT);
  assign _zz_356_ = execute_SRC_LESS;
  assign _zz_357_ = (decode_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_358_ = decode_INSTRUCTION[19 : 15];
  assign _zz_359_ = decode_INSTRUCTION[31 : 20];
  assign _zz_360_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_361_ = ($signed(_zz_362_) + $signed(_zz_365_));
  assign _zz_362_ = ($signed(_zz_363_) + $signed(_zz_364_));
  assign _zz_363_ = execute_SRC1;
  assign _zz_364_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_365_ = (execute_SRC_USE_SUB_LESS ? _zz_366_ : _zz_367_);
  assign _zz_366_ = 32'h00000001;
  assign _zz_367_ = 32'h0;
  assign _zz_368_ = {{14{writeBack_MUL_LOW[51]}}, writeBack_MUL_LOW};
  assign _zz_369_ = ({32'd0,writeBack_MUL_HH} <<< 32);
  assign _zz_370_ = writeBack_MUL_LOW[31 : 0];
  assign _zz_371_ = writeBack_MulPlugin_result[63 : 32];
  assign _zz_372_ = memory_MulDivIterativePlugin_div_counter_willIncrement;
  assign _zz_373_ = {5'd0, _zz_372_};
  assign _zz_374_ = {1'd0, memory_MulDivIterativePlugin_rs2};
  assign _zz_375_ = memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator[31:0];
  assign _zz_376_ = memory_MulDivIterativePlugin_div_stage_0_remainderShifted[31:0];
  assign _zz_377_ = {_zz_140_,(! memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator[32])};
  assign _zz_378_ = _zz_379_;
  assign _zz_379_ = _zz_380_;
  assign _zz_380_ = ({1'b0,(memory_MulDivIterativePlugin_div_needRevert ? (~ _zz_141_) : _zz_141_)} + _zz_382_);
  assign _zz_381_ = memory_MulDivIterativePlugin_div_needRevert;
  assign _zz_382_ = {32'd0, _zz_381_};
  assign _zz_383_ = _zz_143_;
  assign _zz_384_ = {32'd0, _zz_383_};
  assign _zz_385_ = _zz_142_;
  assign _zz_386_ = {31'd0, _zz_385_};
  assign _zz_387_ = (_zz_152_ & (~ _zz_388_));
  assign _zz_388_ = (_zz_152_ - (2'b01));
  assign _zz_389_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_390_ = execute_INSTRUCTION[31 : 20];
  assign _zz_391_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_392_ = MmuPlugin_ports_0_entryToReplace_willIncrement;
  assign _zz_393_ = {1'd0, _zz_392_};
  assign _zz_394_ = MmuPlugin_ports_1_entryToReplace_willIncrement;
  assign _zz_395_ = {1'd0, _zz_394_};
  assign _zz_396_ = MmuPlugin_dBusAccess_rsp_payload_data[0 : 0];
  assign _zz_397_ = MmuPlugin_dBusAccess_rsp_payload_data[1 : 1];
  assign _zz_398_ = MmuPlugin_dBusAccess_rsp_payload_data[2 : 2];
  assign _zz_399_ = MmuPlugin_dBusAccess_rsp_payload_data[3 : 3];
  assign _zz_400_ = MmuPlugin_dBusAccess_rsp_payload_data[4 : 4];
  assign _zz_401_ = MmuPlugin_dBusAccess_rsp_payload_data[5 : 5];
  assign _zz_402_ = MmuPlugin_dBusAccess_rsp_payload_data[6 : 6];
  assign _zz_403_ = MmuPlugin_dBusAccess_rsp_payload_data[7 : 7];
  assign _zz_404_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_405_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_406_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_407_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_408_ = execute_CsrPlugin_writeData[19 : 19];
  assign _zz_409_ = execute_CsrPlugin_writeData[18 : 18];
  assign _zz_410_ = execute_CsrPlugin_writeData[17 : 17];
  assign _zz_411_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_412_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_413_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_414_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_415_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_416_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_417_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_418_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_419_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_420_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_421_ = execute_CsrPlugin_writeData[8 : 8];
  assign _zz_422_ = execute_CsrPlugin_writeData[2 : 2];
  assign _zz_423_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_424_ = execute_CsrPlugin_writeData[13 : 13];
  assign _zz_425_ = execute_CsrPlugin_writeData[4 : 4];
  assign _zz_426_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_427_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_428_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_429_ = execute_CsrPlugin_writeData[12 : 12];
  assign _zz_430_ = execute_CsrPlugin_writeData[15 : 15];
  assign _zz_431_ = execute_CsrPlugin_writeData[6 : 6];
  assign _zz_432_ = execute_CsrPlugin_writeData[0 : 0];
  assign _zz_433_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_434_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_435_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_436_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_437_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_438_ = execute_CsrPlugin_writeData[19 : 19];
  assign _zz_439_ = execute_CsrPlugin_writeData[18 : 18];
  assign _zz_440_ = execute_CsrPlugin_writeData[17 : 17];
  assign _zz_441_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_442_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_443_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_444_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_445_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_446_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_447_ = execute_CsrPlugin_writeData[31 : 31];
  assign _zz_448_ = execute_CsrPlugin_writeData[31 : 31];
  assign _zz_449_ = {IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_target,{IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_last2Bytes,{IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_branchWish,IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_source}}};
  assign _zz_450_ = 1'b1;
  assign _zz_451_ = 1'b1;
  assign _zz_452_ = {_zz_56_,_zz_55_};
  assign _zz_453_ = 32'h0000107f;
  assign _zz_454_ = (decode_INSTRUCTION & 32'h0000207f);
  assign _zz_455_ = 32'h00002073;
  assign _zz_456_ = ((decode_INSTRUCTION & 32'h0000407f) == 32'h00004063);
  assign _zz_457_ = ((decode_INSTRUCTION & 32'h0000207f) == 32'h00002013);
  assign _zz_458_ = {((decode_INSTRUCTION & 32'h0000603f) == 32'h00000023),{((decode_INSTRUCTION & 32'h0000207f) == 32'h00000003),{((decode_INSTRUCTION & _zz_459_) == 32'h00000003),{(_zz_460_ == _zz_461_),{_zz_462_,{_zz_463_,_zz_464_}}}}}};
  assign _zz_459_ = 32'h0000505f;
  assign _zz_460_ = (decode_INSTRUCTION & 32'h0000707b);
  assign _zz_461_ = 32'h00000063;
  assign _zz_462_ = ((decode_INSTRUCTION & 32'h0000607f) == 32'h0000000f);
  assign _zz_463_ = ((decode_INSTRUCTION & 32'hfc00007f) == 32'h00000033);
  assign _zz_464_ = {((decode_INSTRUCTION & 32'hf800707f) == 32'h1800202f),{((decode_INSTRUCTION & 32'hbc00707f) == 32'h00005013),{((decode_INSTRUCTION & _zz_465_) == 32'h00001013),{(_zz_466_ == _zz_467_),{_zz_468_,{_zz_469_,_zz_470_}}}}}};
  assign _zz_465_ = 32'hfc00307f;
  assign _zz_466_ = (decode_INSTRUCTION & 32'hbe00707f);
  assign _zz_467_ = 32'h00005033;
  assign _zz_468_ = ((decode_INSTRUCTION & 32'hbe00707f) == 32'h00000033);
  assign _zz_469_ = ((decode_INSTRUCTION & 32'hf9f0707f) == 32'h1000202f);
  assign _zz_470_ = {((decode_INSTRUCTION & 32'hfe007fff) == 32'h12000073),{((decode_INSTRUCTION & 32'hdfffffff) == 32'h10200073),{((decode_INSTRUCTION & 32'hffefffff) == 32'h00000073),((decode_INSTRUCTION & 32'hffffffff) == 32'h10500073)}}};
  assign _zz_471_ = (_zz_62_[11 : 10] == (2'b01));
  assign _zz_472_ = ((_zz_62_[11 : 10] == (2'b11)) && (_zz_62_[6 : 5] == (2'b00)));
  assign _zz_473_ = 7'h0;
  assign _zz_474_ = _zz_62_[6 : 2];
  assign _zz_475_ = _zz_62_[12];
  assign _zz_476_ = _zz_62_[11 : 7];
  assign _zz_477_ = ((decode_INSTRUCTION & 32'h00002030) == 32'h00002010);
  assign _zz_478_ = ((decode_INSTRUCTION & _zz_488_) == 32'h00000010);
  assign _zz_479_ = {(_zz_489_ == _zz_490_),(_zz_491_ == _zz_492_)};
  assign _zz_480_ = (decode_INSTRUCTION & 32'h00001000);
  assign _zz_481_ = 32'h00001000;
  assign _zz_482_ = ((decode_INSTRUCTION & 32'h00003000) == 32'h00002000);
  assign _zz_483_ = {(_zz_493_ == _zz_494_),{_zz_495_,_zz_496_}};
  assign _zz_484_ = (3'b000);
  assign _zz_485_ = ({_zz_497_,{_zz_498_,_zz_499_}} != (3'b000));
  assign _zz_486_ = (_zz_107_ != (1'b0));
  assign _zz_487_ = {(_zz_500_ != _zz_501_),{_zz_502_,{_zz_503_,_zz_504_}}};
  assign _zz_488_ = 32'h00001030;
  assign _zz_489_ = (decode_INSTRUCTION & 32'h02003020);
  assign _zz_490_ = 32'h00000020;
  assign _zz_491_ = (decode_INSTRUCTION & 32'h12002060);
  assign _zz_492_ = 32'h00002020;
  assign _zz_493_ = (decode_INSTRUCTION & 32'h00000034);
  assign _zz_494_ = 32'h00000020;
  assign _zz_495_ = ((decode_INSTRUCTION & _zz_505_) == 32'h00000020);
  assign _zz_496_ = ((decode_INSTRUCTION & _zz_506_) == 32'h08000020);
  assign _zz_497_ = ((decode_INSTRUCTION & _zz_507_) == 32'h00000040);
  assign _zz_498_ = (_zz_508_ == _zz_509_);
  assign _zz_499_ = (_zz_510_ == _zz_511_);
  assign _zz_500_ = {_zz_512_,{_zz_513_,_zz_514_}};
  assign _zz_501_ = 5'h0;
  assign _zz_502_ = ({_zz_515_,_zz_516_} != (2'b00));
  assign _zz_503_ = (_zz_517_ != _zz_518_);
  assign _zz_504_ = {_zz_519_,{_zz_520_,_zz_521_}};
  assign _zz_505_ = 32'h00000064;
  assign _zz_506_ = 32'h08000070;
  assign _zz_507_ = 32'h00000044;
  assign _zz_508_ = (decode_INSTRUCTION & 32'h00002014);
  assign _zz_509_ = 32'h00002010;
  assign _zz_510_ = (decode_INSTRUCTION & 32'h40000034);
  assign _zz_511_ = 32'h40000030;
  assign _zz_512_ = ((decode_INSTRUCTION & _zz_522_) == 32'h00000040);
  assign _zz_513_ = (_zz_523_ == _zz_524_);
  assign _zz_514_ = {_zz_525_,{_zz_526_,_zz_527_}};
  assign _zz_515_ = (_zz_528_ == _zz_529_);
  assign _zz_516_ = (_zz_530_ == _zz_531_);
  assign _zz_517_ = {_zz_532_,{_zz_533_,_zz_534_}};
  assign _zz_518_ = (3'b000);
  assign _zz_519_ = ({_zz_535_,_zz_536_} != 7'h0);
  assign _zz_520_ = (_zz_537_ != _zz_538_);
  assign _zz_521_ = {_zz_539_,{_zz_540_,_zz_541_}};
  assign _zz_522_ = 32'h00000040;
  assign _zz_523_ = (decode_INSTRUCTION & 32'h00004020);
  assign _zz_524_ = 32'h00004020;
  assign _zz_525_ = ((decode_INSTRUCTION & _zz_542_) == 32'h00000010);
  assign _zz_526_ = _zz_111_;
  assign _zz_527_ = (_zz_543_ == _zz_544_);
  assign _zz_528_ = (decode_INSTRUCTION & 32'h00007034);
  assign _zz_529_ = 32'h00005010;
  assign _zz_530_ = (decode_INSTRUCTION & 32'h02007064);
  assign _zz_531_ = 32'h00005020;
  assign _zz_532_ = ((decode_INSTRUCTION & _zz_545_) == 32'h40001010);
  assign _zz_533_ = (_zz_546_ == _zz_547_);
  assign _zz_534_ = (_zz_548_ == _zz_549_);
  assign _zz_535_ = _zz_110_;
  assign _zz_536_ = {_zz_550_,{_zz_551_,_zz_552_}};
  assign _zz_537_ = (_zz_553_ == _zz_554_);
  assign _zz_538_ = (1'b0);
  assign _zz_539_ = (_zz_555_ != (1'b0));
  assign _zz_540_ = (_zz_556_ != _zz_557_);
  assign _zz_541_ = {_zz_558_,{_zz_559_,_zz_560_}};
  assign _zz_542_ = 32'h00000030;
  assign _zz_543_ = (decode_INSTRUCTION & 32'h12000020);
  assign _zz_544_ = 32'h00000020;
  assign _zz_545_ = 32'h40003054;
  assign _zz_546_ = (decode_INSTRUCTION & 32'h00007034);
  assign _zz_547_ = 32'h00001010;
  assign _zz_548_ = (decode_INSTRUCTION & 32'h02007054);
  assign _zz_549_ = 32'h00001010;
  assign _zz_550_ = ((decode_INSTRUCTION & _zz_561_) == 32'h00001010);
  assign _zz_551_ = (_zz_562_ == _zz_563_);
  assign _zz_552_ = {_zz_564_,{_zz_565_,_zz_566_}};
  assign _zz_553_ = (decode_INSTRUCTION & 32'h00004014);
  assign _zz_554_ = 32'h00004010;
  assign _zz_555_ = ((decode_INSTRUCTION & _zz_567_) == 32'h00002010);
  assign _zz_556_ = {_zz_108_,_zz_568_};
  assign _zz_557_ = (2'b00);
  assign _zz_558_ = ({_zz_569_,_zz_570_} != 6'h0);
  assign _zz_559_ = (_zz_571_ != _zz_572_);
  assign _zz_560_ = {_zz_573_,{_zz_574_,_zz_575_}};
  assign _zz_561_ = 32'h00001010;
  assign _zz_562_ = (decode_INSTRUCTION & 32'h00002010);
  assign _zz_563_ = 32'h00002010;
  assign _zz_564_ = ((decode_INSTRUCTION & _zz_576_) == 32'h00002008);
  assign _zz_565_ = (_zz_577_ == _zz_578_);
  assign _zz_566_ = {_zz_111_,_zz_579_};
  assign _zz_567_ = 32'h00006014;
  assign _zz_568_ = ((decode_INSTRUCTION & _zz_580_) == 32'h0);
  assign _zz_569_ = (_zz_581_ == _zz_582_);
  assign _zz_570_ = {_zz_583_,{_zz_584_,_zz_585_}};
  assign _zz_571_ = (_zz_586_ == _zz_587_);
  assign _zz_572_ = (1'b0);
  assign _zz_573_ = (_zz_588_ != (1'b0));
  assign _zz_574_ = (_zz_589_ != _zz_590_);
  assign _zz_575_ = {_zz_591_,{_zz_592_,_zz_593_}};
  assign _zz_576_ = 32'h00002008;
  assign _zz_577_ = (decode_INSTRUCTION & 32'h00000050);
  assign _zz_578_ = 32'h00000010;
  assign _zz_579_ = ((decode_INSTRUCTION & _zz_594_) == 32'h0);
  assign _zz_580_ = 32'h00000058;
  assign _zz_581_ = (decode_INSTRUCTION & 32'h00002040);
  assign _zz_582_ = 32'h00002040;
  assign _zz_583_ = ((decode_INSTRUCTION & _zz_595_) == 32'h00001040);
  assign _zz_584_ = (_zz_596_ == _zz_597_);
  assign _zz_585_ = {_zz_598_,{_zz_599_,_zz_600_}};
  assign _zz_586_ = (decode_INSTRUCTION & 32'h10103050);
  assign _zz_587_ = 32'h00100050;
  assign _zz_588_ = ((decode_INSTRUCTION & _zz_601_) == 32'h02004020);
  assign _zz_589_ = {_zz_110_,_zz_602_};
  assign _zz_590_ = (2'b00);
  assign _zz_591_ = (_zz_603_ != (1'b0));
  assign _zz_592_ = (_zz_604_ != _zz_605_);
  assign _zz_593_ = {_zz_606_,{_zz_607_,_zz_608_}};
  assign _zz_594_ = 32'h00000028;
  assign _zz_595_ = 32'h00001040;
  assign _zz_596_ = (decode_INSTRUCTION & 32'h00000050);
  assign _zz_597_ = 32'h00000040;
  assign _zz_598_ = ((decode_INSTRUCTION & 32'h08002008) == 32'h00002008);
  assign _zz_599_ = ((decode_INSTRUCTION & _zz_609_) == 32'h00000040);
  assign _zz_600_ = ((decode_INSTRUCTION & _zz_610_) == 32'h0);
  assign _zz_601_ = 32'h02004064;
  assign _zz_602_ = ((decode_INSTRUCTION & 32'h0000001c) == 32'h00000004);
  assign _zz_603_ = ((decode_INSTRUCTION & 32'h00000058) == 32'h00000040);
  assign _zz_604_ = ((decode_INSTRUCTION & _zz_611_) == 32'h00000024);
  assign _zz_605_ = (1'b0);
  assign _zz_606_ = ({_zz_612_,_zz_613_} != (2'b00));
  assign _zz_607_ = ({_zz_614_,_zz_615_} != (2'b00));
  assign _zz_608_ = {(_zz_616_ != _zz_617_),{_zz_618_,{_zz_619_,_zz_620_}}};
  assign _zz_609_ = 32'h02100040;
  assign _zz_610_ = 32'h00000038;
  assign _zz_611_ = 32'h00000064;
  assign _zz_612_ = ((decode_INSTRUCTION & 32'h00001050) == 32'h00001050);
  assign _zz_613_ = ((decode_INSTRUCTION & 32'h00002050) == 32'h00002050);
  assign _zz_614_ = ((decode_INSTRUCTION & _zz_621_) == 32'h08000020);
  assign _zz_615_ = ((decode_INSTRUCTION & _zz_622_) == 32'h00000020);
  assign _zz_616_ = ((decode_INSTRUCTION & _zz_623_) == 32'h02000030);
  assign _zz_617_ = (1'b0);
  assign _zz_618_ = ({_zz_110_,{_zz_624_,_zz_625_}} != (3'b000));
  assign _zz_619_ = ({_zz_626_,_zz_627_} != (2'b00));
  assign _zz_620_ = {(_zz_628_ != _zz_629_),{_zz_630_,{_zz_631_,_zz_632_}}};
  assign _zz_621_ = 32'h08000020;
  assign _zz_622_ = 32'h00000028;
  assign _zz_623_ = 32'h02004074;
  assign _zz_624_ = _zz_109_;
  assign _zz_625_ = ((decode_INSTRUCTION & _zz_633_) == 32'h00000004);
  assign _zz_626_ = _zz_109_;
  assign _zz_627_ = ((decode_INSTRUCTION & _zz_634_) == 32'h00000004);
  assign _zz_628_ = {(_zz_635_ == _zz_636_),{_zz_637_,{_zz_638_,_zz_639_}}};
  assign _zz_629_ = 5'h0;
  assign _zz_630_ = (_zz_107_ != (1'b0));
  assign _zz_631_ = ({_zz_640_,_zz_641_} != (2'b00));
  assign _zz_632_ = {(_zz_642_ != _zz_643_),{_zz_644_,{_zz_645_,_zz_646_}}};
  assign _zz_633_ = 32'h00002014;
  assign _zz_634_ = 32'h0000004c;
  assign _zz_635_ = (decode_INSTRUCTION & 32'h00000044);
  assign _zz_636_ = 32'h0;
  assign _zz_637_ = ((decode_INSTRUCTION & 32'h00000018) == 32'h0);
  assign _zz_638_ = ((decode_INSTRUCTION & _zz_647_) == 32'h00002000);
  assign _zz_639_ = {(_zz_648_ == _zz_649_),_zz_108_};
  assign _zz_640_ = _zz_106_;
  assign _zz_641_ = ((decode_INSTRUCTION & _zz_650_) == 32'h00000020);
  assign _zz_642_ = {_zz_106_,(_zz_651_ == _zz_652_)};
  assign _zz_643_ = (2'b00);
  assign _zz_644_ = ({_zz_653_,_zz_654_} != (2'b00));
  assign _zz_645_ = (_zz_655_ != (1'b0));
  assign _zz_646_ = {(_zz_656_ != _zz_657_),{_zz_658_,_zz_659_}};
  assign _zz_647_ = 32'h00006004;
  assign _zz_648_ = (decode_INSTRUCTION & 32'h00005004);
  assign _zz_649_ = 32'h00001000;
  assign _zz_650_ = 32'h00000070;
  assign _zz_651_ = (decode_INSTRUCTION & 32'h00000020);
  assign _zz_652_ = 32'h0;
  assign _zz_653_ = ((decode_INSTRUCTION & 32'h10103050) == 32'h00000050);
  assign _zz_654_ = ((decode_INSTRUCTION & 32'h12203050) == 32'h10000050);
  assign _zz_655_ = ((decode_INSTRUCTION & 32'h02103050) == 32'h00000050);
  assign _zz_656_ = ((decode_INSTRUCTION & 32'h00000008) == 32'h00000008);
  assign _zz_657_ = (1'b0);
  assign _zz_658_ = ({((decode_INSTRUCTION & _zz_660_) == 32'h00002000),((decode_INSTRUCTION & _zz_661_) == 32'h00001000)} != (2'b00));
  assign _zz_659_ = (((decode_INSTRUCTION & 32'h02003050) == 32'h02000050) != (1'b0));
  assign _zz_660_ = 32'h00002010;
  assign _zz_661_ = 32'h00005000;
  assign _zz_662_ = 32'h0;
  always @ (posedge clk) begin
    if(_zz_48_) begin
      IBusSimplePlugin_predictor_history[IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_address] <= _zz_449_;
    end
  end

  always @ (posedge clk) begin
    if(IBusSimplePlugin_iBusRsp_stages_0_output_ready) begin
      _zz_196_ <= IBusSimplePlugin_predictor_history[_zz_329_];
    end
  end

  initial begin
    $readmemb("VexRiscv.v_toplevel_RegFilePlugin_regFile.bin",RegFilePlugin_regFile);
  end
  always @ (posedge clk) begin
    if(_zz_450_) begin
      _zz_197_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge clk) begin
    if(_zz_451_) begin
      _zz_198_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  always @ (posedge clk) begin
    if(_zz_39_) begin
      RegFilePlugin_regFile[lastStageRegFileWrite_payload_address] <= lastStageRegFileWrite_payload_data;
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid            (iBus_rsp_valid                                                  ), //i
    .io_push_ready            (IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready              ), //o
    .io_push_payload_error    (iBus_rsp_payload_error                                          ), //i
    .io_push_payload_inst     (iBus_rsp_payload_inst[31:0]                                     ), //i
    .io_pop_valid             (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid               ), //o
    .io_pop_ready             (_zz_194_                                                        ), //i
    .io_pop_payload_error     (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error       ), //o
    .io_pop_payload_inst      (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst[31:0]  ), //o
    .io_flush                 (_zz_195_                                                        ), //i
    .io_occupancy             (IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy               ), //o
    .clk                      (clk                                                             ), //i
    .reset                    (reset                                                           )  //i
  );
  JtagBridge jtagBridge_1_ ( 
    .io_jtag_tms                       (jtag_tms                                            ), //i
    .io_jtag_tdi                       (jtag_tdi                                            ), //i
    .io_jtag_tdo                       (jtagBridge_1__io_jtag_tdo                           ), //o
    .io_jtag_tck                       (jtag_tck                                            ), //i
    .io_remote_cmd_valid               (jtagBridge_1__io_remote_cmd_valid                   ), //o
    .io_remote_cmd_ready               (systemDebugger_1__io_remote_cmd_ready               ), //i
    .io_remote_cmd_payload_last        (jtagBridge_1__io_remote_cmd_payload_last            ), //o
    .io_remote_cmd_payload_fragment    (jtagBridge_1__io_remote_cmd_payload_fragment        ), //o
    .io_remote_rsp_valid               (systemDebugger_1__io_remote_rsp_valid               ), //i
    .io_remote_rsp_ready               (jtagBridge_1__io_remote_rsp_ready                   ), //o
    .io_remote_rsp_payload_error       (systemDebugger_1__io_remote_rsp_payload_error       ), //i
    .io_remote_rsp_payload_data        (systemDebugger_1__io_remote_rsp_payload_data[31:0]  ), //i
    .clk                               (clk                                                 ), //i
    .debugReset                        (debugReset                                          )  //i
  );
  SystemDebugger systemDebugger_1_ ( 
    .io_remote_cmd_valid               (jtagBridge_1__io_remote_cmd_valid                   ), //i
    .io_remote_cmd_ready               (systemDebugger_1__io_remote_cmd_ready               ), //o
    .io_remote_cmd_payload_last        (jtagBridge_1__io_remote_cmd_payload_last            ), //i
    .io_remote_cmd_payload_fragment    (jtagBridge_1__io_remote_cmd_payload_fragment        ), //i
    .io_remote_rsp_valid               (systemDebugger_1__io_remote_rsp_valid               ), //o
    .io_remote_rsp_ready               (jtagBridge_1__io_remote_rsp_ready                   ), //i
    .io_remote_rsp_payload_error       (systemDebugger_1__io_remote_rsp_payload_error       ), //o
    .io_remote_rsp_payload_data        (systemDebugger_1__io_remote_rsp_payload_data[31:0]  ), //o
    .io_mem_cmd_valid                  (systemDebugger_1__io_mem_cmd_valid                  ), //o
    .io_mem_cmd_ready                  (debug_bus_cmd_ready                                 ), //i
    .io_mem_cmd_payload_address        (systemDebugger_1__io_mem_cmd_payload_address[31:0]  ), //o
    .io_mem_cmd_payload_data           (systemDebugger_1__io_mem_cmd_payload_data[31:0]     ), //o
    .io_mem_cmd_payload_wr             (systemDebugger_1__io_mem_cmd_payload_wr             ), //o
    .io_mem_cmd_payload_size           (systemDebugger_1__io_mem_cmd_payload_size[1:0]      ), //o
    .io_mem_rsp_valid                  (_zz_193_                                            ), //i
    .io_mem_rsp_payload                (debug_bus_rsp_data[31:0]                            ), //i
    .clk                               (clk                                                 ), //i
    .debugReset                        (debugReset                                          )  //i
  );
  always @(*) begin
    case(_zz_452_)
      2'b00 : begin
        _zz_199_ = CsrPlugin_jumpInterface_payload;
      end
      2'b01 : begin
        _zz_199_ = DBusSimplePlugin_redoBranch_payload;
      end
      2'b10 : begin
        _zz_199_ = BranchPlugin_jumpInterface_payload;
      end
      default : begin
        _zz_199_ = CsrPlugin_redoInterface_payload;
      end
    endcase
  end

  always @(*) begin
    case(_zz_168_)
      2'b00 : begin
        _zz_200_ = MmuPlugin_ports_0_cache_0_valid;
        _zz_201_ = MmuPlugin_ports_0_cache_0_exception;
        _zz_202_ = MmuPlugin_ports_0_cache_0_superPage;
        _zz_203_ = MmuPlugin_ports_0_cache_0_virtualAddress_0;
        _zz_204_ = MmuPlugin_ports_0_cache_0_virtualAddress_1;
        _zz_205_ = MmuPlugin_ports_0_cache_0_physicalAddress_0;
        _zz_206_ = MmuPlugin_ports_0_cache_0_physicalAddress_1;
        _zz_207_ = MmuPlugin_ports_0_cache_0_allowRead;
        _zz_208_ = MmuPlugin_ports_0_cache_0_allowWrite;
        _zz_209_ = MmuPlugin_ports_0_cache_0_allowExecute;
        _zz_210_ = MmuPlugin_ports_0_cache_0_allowUser;
      end
      2'b01 : begin
        _zz_200_ = MmuPlugin_ports_0_cache_1_valid;
        _zz_201_ = MmuPlugin_ports_0_cache_1_exception;
        _zz_202_ = MmuPlugin_ports_0_cache_1_superPage;
        _zz_203_ = MmuPlugin_ports_0_cache_1_virtualAddress_0;
        _zz_204_ = MmuPlugin_ports_0_cache_1_virtualAddress_1;
        _zz_205_ = MmuPlugin_ports_0_cache_1_physicalAddress_0;
        _zz_206_ = MmuPlugin_ports_0_cache_1_physicalAddress_1;
        _zz_207_ = MmuPlugin_ports_0_cache_1_allowRead;
        _zz_208_ = MmuPlugin_ports_0_cache_1_allowWrite;
        _zz_209_ = MmuPlugin_ports_0_cache_1_allowExecute;
        _zz_210_ = MmuPlugin_ports_0_cache_1_allowUser;
      end
      2'b10 : begin
        _zz_200_ = MmuPlugin_ports_0_cache_2_valid;
        _zz_201_ = MmuPlugin_ports_0_cache_2_exception;
        _zz_202_ = MmuPlugin_ports_0_cache_2_superPage;
        _zz_203_ = MmuPlugin_ports_0_cache_2_virtualAddress_0;
        _zz_204_ = MmuPlugin_ports_0_cache_2_virtualAddress_1;
        _zz_205_ = MmuPlugin_ports_0_cache_2_physicalAddress_0;
        _zz_206_ = MmuPlugin_ports_0_cache_2_physicalAddress_1;
        _zz_207_ = MmuPlugin_ports_0_cache_2_allowRead;
        _zz_208_ = MmuPlugin_ports_0_cache_2_allowWrite;
        _zz_209_ = MmuPlugin_ports_0_cache_2_allowExecute;
        _zz_210_ = MmuPlugin_ports_0_cache_2_allowUser;
      end
      default : begin
        _zz_200_ = MmuPlugin_ports_0_cache_3_valid;
        _zz_201_ = MmuPlugin_ports_0_cache_3_exception;
        _zz_202_ = MmuPlugin_ports_0_cache_3_superPage;
        _zz_203_ = MmuPlugin_ports_0_cache_3_virtualAddress_0;
        _zz_204_ = MmuPlugin_ports_0_cache_3_virtualAddress_1;
        _zz_205_ = MmuPlugin_ports_0_cache_3_physicalAddress_0;
        _zz_206_ = MmuPlugin_ports_0_cache_3_physicalAddress_1;
        _zz_207_ = MmuPlugin_ports_0_cache_3_allowRead;
        _zz_208_ = MmuPlugin_ports_0_cache_3_allowWrite;
        _zz_209_ = MmuPlugin_ports_0_cache_3_allowExecute;
        _zz_210_ = MmuPlugin_ports_0_cache_3_allowUser;
      end
    endcase
  end

  always @(*) begin
    case(_zz_171_)
      2'b00 : begin
        _zz_211_ = MmuPlugin_ports_1_cache_0_valid;
        _zz_212_ = MmuPlugin_ports_1_cache_0_exception;
        _zz_213_ = MmuPlugin_ports_1_cache_0_superPage;
        _zz_214_ = MmuPlugin_ports_1_cache_0_virtualAddress_0;
        _zz_215_ = MmuPlugin_ports_1_cache_0_virtualAddress_1;
        _zz_216_ = MmuPlugin_ports_1_cache_0_physicalAddress_0;
        _zz_217_ = MmuPlugin_ports_1_cache_0_physicalAddress_1;
        _zz_218_ = MmuPlugin_ports_1_cache_0_allowRead;
        _zz_219_ = MmuPlugin_ports_1_cache_0_allowWrite;
        _zz_220_ = MmuPlugin_ports_1_cache_0_allowExecute;
        _zz_221_ = MmuPlugin_ports_1_cache_0_allowUser;
      end
      2'b01 : begin
        _zz_211_ = MmuPlugin_ports_1_cache_1_valid;
        _zz_212_ = MmuPlugin_ports_1_cache_1_exception;
        _zz_213_ = MmuPlugin_ports_1_cache_1_superPage;
        _zz_214_ = MmuPlugin_ports_1_cache_1_virtualAddress_0;
        _zz_215_ = MmuPlugin_ports_1_cache_1_virtualAddress_1;
        _zz_216_ = MmuPlugin_ports_1_cache_1_physicalAddress_0;
        _zz_217_ = MmuPlugin_ports_1_cache_1_physicalAddress_1;
        _zz_218_ = MmuPlugin_ports_1_cache_1_allowRead;
        _zz_219_ = MmuPlugin_ports_1_cache_1_allowWrite;
        _zz_220_ = MmuPlugin_ports_1_cache_1_allowExecute;
        _zz_221_ = MmuPlugin_ports_1_cache_1_allowUser;
      end
      2'b10 : begin
        _zz_211_ = MmuPlugin_ports_1_cache_2_valid;
        _zz_212_ = MmuPlugin_ports_1_cache_2_exception;
        _zz_213_ = MmuPlugin_ports_1_cache_2_superPage;
        _zz_214_ = MmuPlugin_ports_1_cache_2_virtualAddress_0;
        _zz_215_ = MmuPlugin_ports_1_cache_2_virtualAddress_1;
        _zz_216_ = MmuPlugin_ports_1_cache_2_physicalAddress_0;
        _zz_217_ = MmuPlugin_ports_1_cache_2_physicalAddress_1;
        _zz_218_ = MmuPlugin_ports_1_cache_2_allowRead;
        _zz_219_ = MmuPlugin_ports_1_cache_2_allowWrite;
        _zz_220_ = MmuPlugin_ports_1_cache_2_allowExecute;
        _zz_221_ = MmuPlugin_ports_1_cache_2_allowUser;
      end
      default : begin
        _zz_211_ = MmuPlugin_ports_1_cache_3_valid;
        _zz_212_ = MmuPlugin_ports_1_cache_3_exception;
        _zz_213_ = MmuPlugin_ports_1_cache_3_superPage;
        _zz_214_ = MmuPlugin_ports_1_cache_3_virtualAddress_0;
        _zz_215_ = MmuPlugin_ports_1_cache_3_virtualAddress_1;
        _zz_216_ = MmuPlugin_ports_1_cache_3_physicalAddress_0;
        _zz_217_ = MmuPlugin_ports_1_cache_3_physicalAddress_1;
        _zz_218_ = MmuPlugin_ports_1_cache_3_allowRead;
        _zz_219_ = MmuPlugin_ports_1_cache_3_allowWrite;
        _zz_220_ = MmuPlugin_ports_1_cache_3_allowExecute;
        _zz_221_ = MmuPlugin_ports_1_cache_3_allowUser;
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(decode_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_BRANCH_CTRL_string = "JALR";
      default : decode_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_1__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_1__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_1__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_1__string = "JALR";
      default : _zz_1__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_2_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_2__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_2__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_2__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_2__string = "JALR";
      default : _zz_2__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_3__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_3__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_3__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_3__string = "JALR";
      default : _zz_3__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_4_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_4__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_4__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_4__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_4__string = "SRA_1    ";
      default : _zz_4__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_5__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_5__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_5__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_5__string = "SRA_1    ";
      default : _zz_5__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_6_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_6__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_6__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_6__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_6__string = "SRA_1    ";
      default : _zz_6__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_7_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_7__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_7__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_7__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_7__string = "SRA_1    ";
      default : _zz_7__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_8_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_8__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_8__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_8__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_8__string = "SRA_1    ";
      default : _zz_8__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_ALU_CTRL_string = "BITWISE ";
      default : decode_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_9__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_9__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_9__string = "BITWISE ";
      default : _zz_9__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_10_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_10__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_10__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_10__string = "BITWISE ";
      default : _zz_10__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_11_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_11__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_11__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_11__string = "BITWISE ";
      default : _zz_11__string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_12_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_12__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_12__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_12__string = "AND_1";
      default : _zz_12__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_13_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_13__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_13__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_13__string = "AND_1";
      default : _zz_13__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_14_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_14__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_14__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_14__string = "AND_1";
      default : _zz_14__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_15_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_15__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_15__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_15__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_15__string = "ECALL";
      default : _zz_15__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_16_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_16__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_16__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_16__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_16__string = "ECALL";
      default : _zz_16__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_17_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_17__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_17__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_17__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_17__string = "ECALL";
      default : _zz_17__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_18_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_18__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_18__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_18__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_18__string = "ECALL";
      default : _zz_18__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : decode_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : decode_ENV_CTRL_string = "ECALL";
      default : decode_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_19_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_19__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_19__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_19__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_19__string = "ECALL";
      default : _zz_19__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_20_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_20__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_20__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_20__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_20__string = "ECALL";
      default : _zz_20__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_21_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_21__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_21__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_21__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_21__string = "ECALL";
      default : _zz_21__string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : execute_BRANCH_CTRL_string = "JALR";
      default : execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_22_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_22__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_22__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_22__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_22__string = "JALR";
      default : _zz_22__string = "????";
    endcase
  end
  always @(*) begin
    case(memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : memory_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : memory_ENV_CTRL_string = "ECALL";
      default : memory_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_23_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_23__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_23__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_23__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_23__string = "ECALL";
      default : _zz_23__string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : execute_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : execute_ENV_CTRL_string = "ECALL";
      default : execute_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_24_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_24__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_24__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_24__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_24__string = "ECALL";
      default : _zz_24__string = "?????";
    endcase
  end
  always @(*) begin
    case(writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : writeBack_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : writeBack_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : writeBack_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : writeBack_ENV_CTRL_string = "ECALL";
      default : writeBack_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_25_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_25__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_25__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_25__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_25__string = "ECALL";
      default : _zz_25__string = "?????";
    endcase
  end
  always @(*) begin
    case(memory_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : memory_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : memory_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : memory_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : memory_SHIFT_CTRL_string = "SRA_1    ";
      default : memory_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_28_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_28__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_28__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_28__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_28__string = "SRA_1    ";
      default : _zz_28__string = "?????????";
    endcase
  end
  always @(*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_29_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_29__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_29__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_29__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_29__string = "SRA_1    ";
      default : _zz_29__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_SRC2_CTRL_string = "PC ";
      default : decode_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_32_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_32__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_32__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_32__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_32__string = "PC ";
      default : _zz_32__string = "???";
    endcase
  end
  always @(*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_SRC1_CTRL_string = "URS1        ";
      default : decode_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_34_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_34__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_34__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_34__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_34__string = "URS1        ";
      default : _zz_34__string = "????????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : execute_ALU_CTRL_string = "BITWISE ";
      default : execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_35_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_35__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_35__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_35__string = "BITWISE ";
      default : _zz_35__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_36_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_36__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_36__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_36__string = "AND_1";
      default : _zz_36__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_40_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_40__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_40__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_40__string = "AND_1";
      default : _zz_40__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_41_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_41__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_41__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_41__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_41__string = "SRA_1    ";
      default : _zz_41__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_42_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_42__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_42__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_42__string = "BITWISE ";
      default : _zz_42__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_43_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_43__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_43__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_43__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_43__string = "JALR";
      default : _zz_43__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_44_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_44__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_44__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_44__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_44__string = "URS1        ";
      default : _zz_44__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_45_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_45__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_45__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_45__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_45__string = "PC ";
      default : _zz_45__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_46_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_46__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_46__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_46__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_46__string = "ECALL";
      default : _zz_46__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_112_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_112__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_112__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_112__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_112__string = "ECALL";
      default : _zz_112__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_113_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_113__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_113__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_113__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_113__string = "PC ";
      default : _zz_113__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_114_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_114__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_114__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_114__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_114__string = "URS1        ";
      default : _zz_114__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_115_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_115__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_115__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_115__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_115__string = "JALR";
      default : _zz_115__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_116_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_116__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_116__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_116__string = "BITWISE ";
      default : _zz_116__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_117_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_117__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_117__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_117__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_117__string = "SRA_1    ";
      default : _zz_117__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_118_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_118__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_118__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_118__string = "AND_1";
      default : _zz_118__string = "?????";
    endcase
  end
  always @(*) begin
    case(MmuPlugin_shared_state_1_)
      `MmuPlugin_shared_State_defaultEncoding_IDLE : MmuPlugin_shared_state_1__string = "IDLE  ";
      `MmuPlugin_shared_State_defaultEncoding_L1_CMD : MmuPlugin_shared_state_1__string = "L1_CMD";
      `MmuPlugin_shared_State_defaultEncoding_L1_RSP : MmuPlugin_shared_state_1__string = "L1_RSP";
      `MmuPlugin_shared_State_defaultEncoding_L0_CMD : MmuPlugin_shared_state_1__string = "L0_CMD";
      `MmuPlugin_shared_State_defaultEncoding_L0_RSP : MmuPlugin_shared_state_1__string = "L0_RSP";
      default : MmuPlugin_shared_state_1__string = "??????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_to_execute_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_to_execute_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : decode_to_execute_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : decode_to_execute_ENV_CTRL_string = "ECALL";
      default : decode_to_execute_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_to_memory_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_to_memory_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : execute_to_memory_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : execute_to_memory_ENV_CTRL_string = "ECALL";
      default : execute_to_memory_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(memory_to_writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_to_writeBack_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_to_writeBack_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : memory_to_writeBack_ENV_CTRL_string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : memory_to_writeBack_ENV_CTRL_string = "ECALL";
      default : memory_to_writeBack_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_to_execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_to_execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_to_execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_to_execute_ALU_CTRL_string = "BITWISE ";
      default : decode_to_execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_to_execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_to_execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_to_execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_to_execute_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_to_execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_to_memory_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_to_memory_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_to_memory_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_to_memory_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_to_memory_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_to_execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_to_execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_to_execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_to_execute_BRANCH_CTRL_string = "JALR";
      default : decode_to_execute_BRANCH_CTRL_string = "????";
    endcase
  end
  `endif

  assign decode_SRC2_FORCE_ZERO = (decode_SRC_ADD_ZERO && (! decode_SRC_USE_SUB_LESS));
  assign decode_DO_EBREAK = (((! DebugPlugin_haltIt) && (decode_IS_EBREAK || 1'b0)) && DebugPlugin_allowEBreak);
  assign memory_ATOMIC_HIT = execute_to_memory_ATOMIC_HIT;
  assign decode_BRANCH_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign _zz_4_ = _zz_5_;
  assign decode_SHIFT_CTRL = _zz_6_;
  assign _zz_7_ = _zz_8_;
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_120_;
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_51_;
  assign decode_ALU_CTRL = _zz_9_;
  assign _zz_10_ = _zz_11_;
  assign memory_IS_SFENCE_VMA = execute_to_memory_IS_SFENCE_VMA;
  assign execute_IS_SFENCE_VMA = decode_to_execute_IS_SFENCE_VMA;
  assign decode_IS_SFENCE_VMA = _zz_287_[0];
  assign execute_MUL_LH = ($signed(execute_MulPlugin_aSLow) * $signed(execute_MulPlugin_bHigh));
  assign decode_SRC_LESS_UNSIGNED = _zz_288_[0];
  assign decode_MEMORY_STORE = _zz_289_[0];
  assign memory_MUL_HH = execute_to_memory_MUL_HH;
  assign execute_MUL_HH = ($signed(execute_MulPlugin_aHigh) * $signed(execute_MulPlugin_bHigh));
  assign memory_IS_MUL = execute_to_memory_IS_MUL;
  assign execute_IS_MUL = decode_to_execute_IS_MUL;
  assign decode_IS_MUL = _zz_290_[0];
  assign decode_IS_CSR = _zz_291_[0];
  assign execute_PREDICTION_CONTEXT_hazard = decode_to_execute_PREDICTION_CONTEXT_hazard;
  assign execute_PREDICTION_CONTEXT_hit = decode_to_execute_PREDICTION_CONTEXT_hit;
  assign execute_PREDICTION_CONTEXT_line_source = decode_to_execute_PREDICTION_CONTEXT_line_source;
  assign execute_PREDICTION_CONTEXT_line_branchWish = decode_to_execute_PREDICTION_CONTEXT_line_branchWish;
  assign execute_PREDICTION_CONTEXT_line_last2Bytes = decode_to_execute_PREDICTION_CONTEXT_line_last2Bytes;
  assign execute_PREDICTION_CONTEXT_line_target = decode_to_execute_PREDICTION_CONTEXT_line_target;
  assign decode_PREDICTION_CONTEXT_hazard = IBusSimplePlugin_predictor_injectorContext_hazard;
  assign decode_PREDICTION_CONTEXT_hit = IBusSimplePlugin_predictor_injectorContext_hit;
  assign decode_PREDICTION_CONTEXT_line_source = IBusSimplePlugin_predictor_injectorContext_line_source;
  assign decode_PREDICTION_CONTEXT_line_branchWish = IBusSimplePlugin_predictor_injectorContext_line_branchWish;
  assign decode_PREDICTION_CONTEXT_line_last2Bytes = IBusSimplePlugin_predictor_injectorContext_line_last2Bytes;
  assign decode_PREDICTION_CONTEXT_line_target = IBusSimplePlugin_predictor_injectorContext_line_target;
  assign decode_ALU_BITWISE_CTRL = _zz_12_;
  assign _zz_13_ = _zz_14_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_292_[0];
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_293_[0];
  assign memory_MEMORY_ATOMIC = execute_to_memory_MEMORY_ATOMIC;
  assign decode_MEMORY_ATOMIC = _zz_294_[0];
  assign _zz_15_ = _zz_16_;
  assign _zz_17_ = _zz_18_;
  assign decode_ENV_CTRL = _zz_19_;
  assign _zz_20_ = _zz_21_;
  assign memory_MUL_LOW = ($signed(_zz_295_) + $signed(_zz_303_));
  assign decode_CSR_WRITE_OPCODE = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == 5'h0)) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == 5'h0))));
  assign execute_NEXT_PC2 = (execute_PC + _zz_305_);
  assign execute_SHIFT_RIGHT = _zz_307_;
  assign execute_MUL_HL = ($signed(execute_MulPlugin_aHigh) * $signed(execute_MulPlugin_bSLow));
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = dBus_cmd_payload_address[1 : 0];
  assign decode_IS_RS1_SIGNED = _zz_309_[0];
  assign decode_IS_RS2_SIGNED = _zz_310_[0];
  assign decode_CSR_READ_OPCODE = (decode_INSTRUCTION[13 : 7] != 7'h20);
  assign execute_BRANCH_DO = _zz_158_;
  assign execute_TARGET_MISSMATCH2 = (decode_PC != execute_BRANCH_CALC);
  assign decode_SRC2 = _zz_126_;
  assign decode_IS_DIV = _zz_311_[0];
  assign execute_MUL_LL = (execute_MulPlugin_aULow * execute_MulPlugin_bULow);
  assign decode_SRC1 = _zz_121_;
  assign memory_MEMORY_READ_DATA = dBus_rsp_data;
  assign writeBack_IS_SFENCE_VMA = memory_to_writeBack_IS_SFENCE_VMA;
  assign memory_NEXT_PC2 = execute_to_memory_NEXT_PC2;
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_TARGET_MISSMATCH2 = execute_to_memory_TARGET_MISSMATCH2;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_BRANCH_CALC = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign execute_IS_RVC = decode_to_execute_IS_RVC;
  assign execute_BRANCH_SRC22 = _zz_165_;
  assign execute_BRANCH_CTRL = _zz_22_;
  assign execute_PC = decode_to_execute_PC;
  assign execute_DO_EBREAK = decode_to_execute_DO_EBREAK;
  assign decode_IS_EBREAK = _zz_312_[0];
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_23_;
  assign execute_ENV_CTRL = _zz_24_;
  assign writeBack_ENV_CTRL = _zz_25_;
  assign execute_IS_RS1_SIGNED = decode_to_execute_IS_RS1_SIGNED;
  assign execute_IS_DIV = decode_to_execute_IS_DIV;
  assign execute_IS_RS2_SIGNED = decode_to_execute_IS_RS2_SIGNED;
  assign memory_IS_DIV = execute_to_memory_IS_DIV;
  assign writeBack_IS_MUL = memory_to_writeBack_IS_MUL;
  assign writeBack_MUL_HH = memory_to_writeBack_MUL_HH;
  assign writeBack_MUL_LOW = memory_to_writeBack_MUL_LOW;
  assign memory_MUL_HL = execute_to_memory_MUL_HL;
  assign memory_MUL_LH = execute_to_memory_MUL_LH;
  assign memory_MUL_LL = execute_to_memory_MUL_LL;
  assign execute_RS1 = decode_to_execute_RS1;
  assign decode_RS2_USE = _zz_313_[0];
  assign decode_RS1_USE = _zz_314_[0];
  always @ (*) begin
    _zz_26_ = execute_REGFILE_WRITE_DATA;
    if(_zz_222_)begin
      _zz_26_ = execute_CsrPlugin_readData;
    end
  end

  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    decode_RS2 = decode_RegFilePlugin_rs2Data;
    if(_zz_131_)begin
      if((_zz_132_ == decode_INSTRUCTION[24 : 20]))begin
        decode_RS2 = _zz_133_;
      end
    end
    if(_zz_223_)begin
      if(_zz_224_)begin
        if(_zz_135_)begin
          decode_RS2 = _zz_47_;
        end
      end
    end
    if(_zz_225_)begin
      if(memory_BYPASSABLE_MEMORY_STAGE)begin
        if(_zz_137_)begin
          decode_RS2 = _zz_27_;
        end
      end
    end
    if(_zz_226_)begin
      if(execute_BYPASSABLE_EXECUTE_STAGE)begin
        if(_zz_139_)begin
          decode_RS2 = _zz_26_;
        end
      end
    end
  end

  always @ (*) begin
    decode_RS1 = decode_RegFilePlugin_rs1Data;
    if(_zz_131_)begin
      if((_zz_132_ == decode_INSTRUCTION[19 : 15]))begin
        decode_RS1 = _zz_133_;
      end
    end
    if(_zz_223_)begin
      if(_zz_224_)begin
        if(_zz_134_)begin
          decode_RS1 = _zz_47_;
        end
      end
    end
    if(_zz_225_)begin
      if(memory_BYPASSABLE_MEMORY_STAGE)begin
        if(_zz_136_)begin
          decode_RS1 = _zz_27_;
        end
      end
    end
    if(_zz_226_)begin
      if(execute_BYPASSABLE_EXECUTE_STAGE)begin
        if(_zz_138_)begin
          decode_RS1 = _zz_26_;
        end
      end
    end
  end

  assign memory_SHIFT_RIGHT = execute_to_memory_SHIFT_RIGHT;
  always @ (*) begin
    _zz_27_ = memory_REGFILE_WRITE_DATA;
    if(memory_arbitration_isValid)begin
      case(memory_SHIFT_CTRL)
        `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
          _zz_27_ = _zz_128_;
        end
        `ShiftCtrlEnum_defaultEncoding_SRL_1, `ShiftCtrlEnum_defaultEncoding_SRA_1 : begin
          _zz_27_ = memory_SHIFT_RIGHT;
        end
        default : begin
        end
      endcase
    end
    if(_zz_227_)begin
      _zz_27_ = memory_MulDivIterativePlugin_div_result;
    end
  end

  assign memory_SHIFT_CTRL = _zz_28_;
  assign execute_SHIFT_CTRL = _zz_29_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC2_FORCE_ZERO = decode_to_execute_SRC2_FORCE_ZERO;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_30_ = decode_PC;
  assign _zz_31_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_32_;
  assign _zz_33_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_34_;
  assign decode_SRC_USE_SUB_LESS = _zz_315_[0];
  assign decode_SRC_ADD_ZERO = _zz_316_[0];
  assign execute_SRC_ADD_SUB = execute_SrcPlugin_addSub;
  assign execute_SRC_LESS = execute_SrcPlugin_less;
  assign execute_ALU_CTRL = _zz_35_;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_ALU_BITWISE_CTRL = _zz_36_;
  assign _zz_37_ = writeBack_INSTRUCTION;
  assign _zz_38_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_39_ = 1'b0;
    if(lastStageRegFileWrite_valid)begin
      _zz_39_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_decompressor_output_payload_rsp_inst);
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_317_[0];
    if((decode_INSTRUCTION[11 : 7] == 5'h0))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  assign decode_LEGAL_INSTRUCTION = ({((decode_INSTRUCTION & 32'h0000005f) == 32'h00000017),{((decode_INSTRUCTION & 32'h0000007f) == 32'h0000006f),{((decode_INSTRUCTION & 32'h0000106f) == 32'h00000003),{((decode_INSTRUCTION & _zz_453_) == 32'h00001073),{(_zz_454_ == _zz_455_),{_zz_456_,{_zz_457_,_zz_458_}}}}}}} != 23'h0);
  assign writeBack_ATOMIC_HIT = memory_to_writeBack_ATOMIC_HIT;
  assign writeBack_MEMORY_STORE = memory_to_writeBack_MEMORY_STORE;
  assign writeBack_MEMORY_ATOMIC = memory_to_writeBack_MEMORY_ATOMIC;
  always @ (*) begin
    _zz_47_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_47_ = writeBack_DBusSimplePlugin_rspFormated;
      if((writeBack_MEMORY_ATOMIC && writeBack_MEMORY_STORE))begin
        _zz_47_ = {31'd0, _zz_355_};
      end
    end
    if((writeBack_arbitration_isValid && writeBack_IS_MUL))begin
      case(_zz_285_)
        2'b00 : begin
          _zz_47_ = _zz_370_;
        end
        default : begin
          _zz_47_ = _zz_371_;
        end
      endcase
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_MMU_FAULT = execute_to_memory_MMU_FAULT;
  assign memory_MMU_RSP_physicalAddress = execute_to_memory_MMU_RSP_physicalAddress;
  assign memory_MMU_RSP_isIoAccess = execute_to_memory_MMU_RSP_isIoAccess;
  assign memory_MMU_RSP_isPaging = execute_to_memory_MMU_RSP_isPaging;
  assign memory_MMU_RSP_allowRead = execute_to_memory_MMU_RSP_allowRead;
  assign memory_MMU_RSP_allowWrite = execute_to_memory_MMU_RSP_allowWrite;
  assign memory_MMU_RSP_allowExecute = execute_to_memory_MMU_RSP_allowExecute;
  assign memory_MMU_RSP_exception = execute_to_memory_MMU_RSP_exception;
  assign memory_MMU_RSP_refilling = execute_to_memory_MMU_RSP_refilling;
  assign memory_ALIGNEMENT_FAULT = execute_to_memory_ALIGNEMENT_FAULT;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign memory_MEMORY_STORE = execute_to_memory_MEMORY_STORE;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_ATOMIC_HIT = execute_DBusSimplePlugin_atomic_reserved;
  assign execute_MEMORY_ATOMIC = decode_to_execute_MEMORY_ATOMIC;
  assign execute_MMU_FAULT = ((execute_MMU_RSP_exception || ((! execute_MMU_RSP_allowWrite) && execute_MEMORY_STORE)) || ((! execute_MMU_RSP_allowRead) && (! execute_MEMORY_STORE)));
  assign execute_MMU_RSP_physicalAddress = DBusSimplePlugin_mmuBus_rsp_physicalAddress;
  assign execute_MMU_RSP_isIoAccess = DBusSimplePlugin_mmuBus_rsp_isIoAccess;
  assign execute_MMU_RSP_isPaging = DBusSimplePlugin_mmuBus_rsp_isPaging;
  assign execute_MMU_RSP_allowRead = DBusSimplePlugin_mmuBus_rsp_allowRead;
  assign execute_MMU_RSP_allowWrite = DBusSimplePlugin_mmuBus_rsp_allowWrite;
  assign execute_MMU_RSP_allowExecute = DBusSimplePlugin_mmuBus_rsp_allowExecute;
  assign execute_MMU_RSP_exception = DBusSimplePlugin_mmuBus_rsp_exception;
  assign execute_MMU_RSP_refilling = DBusSimplePlugin_mmuBus_rsp_refilling;
  assign execute_SRC_ADD = execute_SrcPlugin_addSub;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_MEMORY_STORE = decode_to_execute_MEMORY_STORE;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  assign execute_ALIGNEMENT_FAULT = (((dBus_cmd_payload_size == (2'b10)) && (dBus_cmd_payload_address[1 : 0] != (2'b00))) || ((dBus_cmd_payload_size == (2'b01)) && (dBus_cmd_payload_address[0 : 0] != (1'b0))));
  assign decode_MEMORY_ENABLE = _zz_318_[0];
  assign memory_IS_RVC = execute_to_memory_IS_RVC;
  assign memory_PC = execute_to_memory_PC;
  assign memory_PREDICTION_CONTEXT_hazard = execute_to_memory_PREDICTION_CONTEXT_hazard;
  assign memory_PREDICTION_CONTEXT_hit = execute_to_memory_PREDICTION_CONTEXT_hit;
  assign memory_PREDICTION_CONTEXT_line_source = execute_to_memory_PREDICTION_CONTEXT_line_source;
  assign memory_PREDICTION_CONTEXT_line_branchWish = execute_to_memory_PREDICTION_CONTEXT_line_branchWish;
  assign memory_PREDICTION_CONTEXT_line_last2Bytes = execute_to_memory_PREDICTION_CONTEXT_line_last2Bytes;
  assign memory_PREDICTION_CONTEXT_line_target = execute_to_memory_PREDICTION_CONTEXT_line_target;
  always @ (*) begin
    _zz_48_ = 1'b0;
    if(IBusSimplePlugin_predictor_historyWriteDelayPatched_valid)begin
      _zz_48_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_49_ = execute_FORMAL_PC_NEXT;
    if(CsrPlugin_redoInterface_valid)begin
      _zz_49_ = CsrPlugin_redoInterface_payload;
    end
  end

  always @ (*) begin
    _zz_50_ = memory_FORMAL_PC_NEXT;
    if(DBusSimplePlugin_redoBranch_valid)begin
      _zz_50_ = DBusSimplePlugin_redoBranch_payload;
    end
    if(BranchPlugin_jumpInterface_valid)begin
      _zz_50_ = BranchPlugin_jumpInterface_payload;
    end
  end

  assign decode_PC = IBusSimplePlugin_decodePc_pcReg;
  assign decode_INSTRUCTION = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  assign decode_IS_RVC = IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    if(((DBusSimplePlugin_mmuBus_busy && decode_arbitration_isValid) && decode_MEMORY_ENABLE))begin
      decode_arbitration_haltItself = 1'b1;
    end
    case(_zz_104_)
      2'b00 : begin
        if(MmuPlugin_dBusAccess_cmd_valid)begin
          decode_arbitration_haltItself = 1'b1;
        end
      end
      2'b01 : begin
        decode_arbitration_haltItself = 1'b1;
      end
      2'b10 : begin
        decode_arbitration_haltItself = 1'b1;
      end
      default : begin
      end
    endcase
    case(_zz_172_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_haltItself = 1'b1;
      end
      3'b011 : begin
      end
      3'b100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((decode_arbitration_isValid && (_zz_129_ || _zz_130_)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(CsrPlugin_pipelineLiberator_active)begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(({(writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),{(memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),(execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))}} != (3'b000)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(_zz_228_)begin
      decode_arbitration_removeIt = 1'b1;
    end
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_flushIt = 1'b0;
  always @ (*) begin
    decode_arbitration_flushNext = 1'b0;
    if(_zz_228_)begin
      decode_arbitration_flushNext = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_97_)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if(_zz_229_)begin
      if((! execute_CsrPlugin_wfiWake))begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
    if(_zz_222_)begin
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
  end

  always @ (*) begin
    execute_arbitration_haltByOther = 1'b0;
    if(_zz_230_)begin
      execute_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_removeIt = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_flushIt = 1'b0;
    if(_zz_230_)begin
      if(_zz_231_)begin
        execute_arbitration_flushIt = 1'b1;
      end
    end
  end

  always @ (*) begin
    execute_arbitration_flushNext = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_flushNext = 1'b1;
    end
    if(_zz_230_)begin
      if(_zz_231_)begin
        execute_arbitration_flushNext = 1'b1;
      end
    end
    if(_zz_155_)begin
      execute_arbitration_flushNext = 1'b1;
    end
    if(execute_CsrPlugin_csr_384)begin
      if(execute_CsrPlugin_writeEnable)begin
        execute_arbitration_flushNext = 1'b1;
      end
    end
  end

  always @ (*) begin
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_MEMORY_STORE)) && ((! dBus_rsp_ready) || 1'b0)))begin
      memory_arbitration_haltItself = 1'b1;
    end
    if(_zz_227_)begin
      if(((! memory_MulDivIterativePlugin_frontendOk) || (! memory_MulDivIterativePlugin_div_done)))begin
        memory_arbitration_haltItself = 1'b1;
      end
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(DBusSimplePlugin_memoryExceptionPort_valid)begin
      memory_arbitration_removeIt = 1'b1;
    end
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushIt = 1'b0;
    if(DBusSimplePlugin_redoBranch_valid)begin
      memory_arbitration_flushIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushNext = 1'b0;
    if(DBusSimplePlugin_redoBranch_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
    if(DBusSimplePlugin_memoryExceptionPort_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
    if(BranchPlugin_jumpInterface_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
  end

  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushIt = 1'b0;
  always @ (*) begin
    writeBack_arbitration_flushNext = 1'b0;
    if(_zz_232_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
    if(_zz_233_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
  end

  assign lastStageInstruction = writeBack_INSTRUCTION;
  assign lastStagePc = writeBack_PC;
  assign lastStageIsValid = writeBack_arbitration_isValid;
  assign lastStageIsFiring = writeBack_arbitration_isFiring;
  always @ (*) begin
    IBusSimplePlugin_fetcherHalt = 1'b0;
    if(({CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack,{CsrPlugin_exceptionPortCtrl_exceptionValids_memory,{CsrPlugin_exceptionPortCtrl_exceptionValids_execute,CsrPlugin_exceptionPortCtrl_exceptionValids_decode}}} != (4'b0000)))begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_232_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_233_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_230_)begin
      if(_zz_231_)begin
        IBusSimplePlugin_fetcherHalt = 1'b1;
      end
    end
    if(DebugPlugin_haltIt)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_234_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_incomingInstruction = 1'b0;
    if(IBusSimplePlugin_iBusRsp_stages_1_input_valid)begin
      IBusSimplePlugin_incomingInstruction = 1'b1;
    end
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_incomingInstruction = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_inWfi = 1'b0;
    if(_zz_229_)begin
      CsrPlugin_inWfi = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_thirdPartyWake = 1'b0;
    if(DebugPlugin_haltIt)begin
      CsrPlugin_thirdPartyWake = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_valid = 1'b0;
    if(_zz_232_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
    if(_zz_233_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_payload = 32'h0;
    if(_zz_232_)begin
      CsrPlugin_jumpInterface_payload = {CsrPlugin_xtvec_base,(2'b00)};
    end
    if(_zz_233_)begin
      case(_zz_235_)
        2'b11 : begin
          CsrPlugin_jumpInterface_payload = CsrPlugin_mepc;
        end
        2'b01 : begin
          CsrPlugin_jumpInterface_payload = CsrPlugin_sepc;
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    CsrPlugin_forceMachineWire = 1'b0;
    if(DebugPlugin_godmode)begin
      CsrPlugin_forceMachineWire = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_allowInterrupts = 1'b1;
    if((DebugPlugin_haltIt || DebugPlugin_stepIt))begin
      CsrPlugin_allowInterrupts = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_allowException = 1'b1;
    if(DebugPlugin_godmode)begin
      CsrPlugin_allowException = 1'b0;
    end
  end

  assign IBusSimplePlugin_externalFlush = ({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,{execute_arbitration_flushNext,decode_arbitration_flushNext}}} != (4'b0000));
  assign IBusSimplePlugin_jump_pcLoad_valid = ({BranchPlugin_jumpInterface_valid,{CsrPlugin_redoInterface_valid,{CsrPlugin_jumpInterface_valid,DBusSimplePlugin_redoBranch_valid}}} != (4'b0000));
  assign _zz_52_ = {CsrPlugin_redoInterface_valid,{BranchPlugin_jumpInterface_valid,{DBusSimplePlugin_redoBranch_valid,CsrPlugin_jumpInterface_valid}}};
  assign _zz_53_ = (_zz_52_ & (~ _zz_319_));
  assign _zz_54_ = _zz_53_[3];
  assign _zz_55_ = (_zz_53_[1] || _zz_54_);
  assign _zz_56_ = (_zz_53_[2] || _zz_54_);
  assign IBusSimplePlugin_jump_pcLoad_payload = _zz_199_;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_correction = 1'b0;
    if(IBusSimplePlugin_fetchPc_predictionPcLoad_valid)begin
      IBusSimplePlugin_fetchPc_correction = 1'b1;
    end
    if(IBusSimplePlugin_fetchPc_redo_valid)begin
      IBusSimplePlugin_fetchPc_correction = 1'b1;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_correction = 1'b1;
    end
  end

  assign IBusSimplePlugin_fetchPc_corrected = (IBusSimplePlugin_fetchPc_correction || IBusSimplePlugin_fetchPc_correctionReg);
  assign IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b0;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_321_);
    if(IBusSimplePlugin_fetchPc_inc)begin
      IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
    end
    if(IBusSimplePlugin_fetchPc_predictionPcLoad_valid)begin
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_fetchPc_predictionPcLoad_payload;
    end
    if(IBusSimplePlugin_fetchPc_redo_valid)begin
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_fetchPc_redo_payload;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_flushed = 1'b0;
    if(IBusSimplePlugin_fetchPc_redo_valid)begin
      IBusSimplePlugin_fetchPc_flushed = 1'b1;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_flushed = 1'b1;
    end
  end

  assign IBusSimplePlugin_fetchPc_output_valid = ((! IBusSimplePlugin_fetcherHalt) && IBusSimplePlugin_fetchPc_booted);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_pc;
  always @ (*) begin
    IBusSimplePlugin_decodePc_flushed = 1'b0;
    if(_zz_236_)begin
      IBusSimplePlugin_decodePc_flushed = 1'b1;
    end
  end

  assign IBusSimplePlugin_decodePc_pcPlus = (IBusSimplePlugin_decodePc_pcReg + _zz_323_);
  always @ (*) begin
    IBusSimplePlugin_decodePc_injectedDecode = 1'b0;
    if((_zz_172_ != (3'b000)))begin
      IBusSimplePlugin_decodePc_injectedDecode = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_iBusRsp_redoFetch = 1'b0;
    if(IBusSimplePlugin_predictor_compressor_unalignedWordIssue)begin
      IBusSimplePlugin_iBusRsp_redoFetch = 1'b1;
    end
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_mmu_joinCtx_refilling))begin
      IBusSimplePlugin_iBusRsp_redoFetch = 1'b1;
    end
  end

  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_0_input_valid && ((! IBusSimplePlugin_cmdFork_canEmit) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b1;
    end
    if(IBusSimplePlugin_iBusRsp_stages_0_input_valid)begin
      if(IBusSimplePlugin_mmuBus_rsp_refilling)begin
        IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b1;
      end
      if(IBusSimplePlugin_mmuBus_rsp_exception)begin
        IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
      end
    end
  end

  assign _zz_57_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_57_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_57_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
  assign _zz_58_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_58_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_58_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_fetchPc_redo_valid = IBusSimplePlugin_iBusRsp_redoFetch;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_redo_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
    if(IBusSimplePlugin_decompressor_throw2BytesReg)begin
      IBusSimplePlugin_fetchPc_redo_payload[1] = 1'b1;
    end
  end

  assign IBusSimplePlugin_iBusRsp_flush = (IBusSimplePlugin_externalFlush || IBusSimplePlugin_iBusRsp_redoFetch);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = ((1'b0 && (! _zz_59_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_59_ = _zz_60_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_59_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = _zz_61_;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusSimplePlugin_decompressor_input_valid = (IBusSimplePlugin_iBusRsp_output_valid && (! IBusSimplePlugin_iBusRsp_redoFetch));
  assign IBusSimplePlugin_decompressor_input_payload_pc = IBusSimplePlugin_iBusRsp_output_payload_pc;
  assign IBusSimplePlugin_decompressor_input_payload_rsp_error = IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
  assign IBusSimplePlugin_decompressor_input_payload_rsp_inst = IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
  assign IBusSimplePlugin_decompressor_input_payload_isRvc = IBusSimplePlugin_iBusRsp_output_payload_isRvc;
  assign IBusSimplePlugin_iBusRsp_output_ready = IBusSimplePlugin_decompressor_input_ready;
  assign IBusSimplePlugin_decompressor_flushNext = 1'b0;
  assign IBusSimplePlugin_decompressor_consumeCurrent = 1'b0;
  assign IBusSimplePlugin_decompressor_isInputLowRvc = (IBusSimplePlugin_decompressor_input_payload_rsp_inst[1 : 0] != (2'b11));
  assign IBusSimplePlugin_decompressor_isInputHighRvc = (IBusSimplePlugin_decompressor_input_payload_rsp_inst[17 : 16] != (2'b11));
  assign IBusSimplePlugin_decompressor_throw2Bytes = (IBusSimplePlugin_decompressor_throw2BytesReg || IBusSimplePlugin_decompressor_input_payload_pc[1]);
  assign IBusSimplePlugin_decompressor_unaligned = (IBusSimplePlugin_decompressor_throw2Bytes || IBusSimplePlugin_decompressor_bufferValid);
  assign IBusSimplePlugin_decompressor_raw = (IBusSimplePlugin_decompressor_bufferValid ? {IBusSimplePlugin_decompressor_input_payload_rsp_inst[15 : 0],IBusSimplePlugin_decompressor_bufferData} : {IBusSimplePlugin_decompressor_input_payload_rsp_inst[31 : 16],(IBusSimplePlugin_decompressor_throw2Bytes ? IBusSimplePlugin_decompressor_input_payload_rsp_inst[31 : 16] : IBusSimplePlugin_decompressor_input_payload_rsp_inst[15 : 0])});
  assign IBusSimplePlugin_decompressor_isRvc = (IBusSimplePlugin_decompressor_raw[1 : 0] != (2'b11));
  assign _zz_62_ = IBusSimplePlugin_decompressor_raw[15 : 0];
  always @ (*) begin
    IBusSimplePlugin_decompressor_decompressed = 32'h0;
    case(_zz_281_)
      5'b00000 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{{{(2'b00),_zz_62_[10 : 7]},_zz_62_[12 : 11]},_zz_62_[5]},_zz_62_[6]},(2'b00)},5'h02},(3'b000)},_zz_64_},7'h13};
      end
      5'b00010 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{_zz_65_,_zz_63_},(3'b010)},_zz_64_},7'h03};
      end
      5'b00110 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_65_[11 : 5],_zz_64_},_zz_63_},(3'b010)},_zz_65_[4 : 0]},7'h23};
      end
      5'b01000 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{_zz_67_,_zz_62_[11 : 7]},(3'b000)},_zz_62_[11 : 7]},7'h13};
      end
      5'b01001 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_70_[20],_zz_70_[10 : 1]},_zz_70_[11]},_zz_70_[19 : 12]},_zz_82_},7'h6f};
      end
      5'b01010 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{_zz_67_,5'h0},(3'b000)},_zz_62_[11 : 7]},7'h13};
      end
      5'b01011 : begin
        IBusSimplePlugin_decompressor_decompressed = ((_zz_62_[11 : 7] == 5'h02) ? {{{{{{{{{_zz_74_,_zz_62_[4 : 3]},_zz_62_[5]},_zz_62_[2]},_zz_62_[6]},(4'b0000)},_zz_62_[11 : 7]},(3'b000)},_zz_62_[11 : 7]},7'h13} : {{_zz_324_[31 : 12],_zz_62_[11 : 7]},7'h37});
      end
      5'b01100 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{((_zz_62_[11 : 10] == (2'b10)) ? _zz_88_ : {{(1'b0),(_zz_471_ || _zz_472_)},5'h0}),(((! _zz_62_[11]) || _zz_84_) ? _zz_62_[6 : 2] : _zz_64_)},_zz_63_},_zz_86_},_zz_63_},(_zz_84_ ? 7'h13 : 7'h33)};
      end
      5'b01101 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_77_[20],_zz_77_[10 : 1]},_zz_77_[11]},_zz_77_[19 : 12]},_zz_81_},7'h6f};
      end
      5'b01110 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{_zz_80_[12],_zz_80_[10 : 5]},_zz_81_},_zz_63_},(3'b000)},_zz_80_[4 : 1]},_zz_80_[11]},7'h63};
      end
      5'b01111 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{_zz_80_[12],_zz_80_[10 : 5]},_zz_81_},_zz_63_},(3'b001)},_zz_80_[4 : 1]},_zz_80_[11]},7'h63};
      end
      5'b10000 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{7'h0,_zz_62_[6 : 2]},_zz_62_[11 : 7]},(3'b001)},_zz_62_[11 : 7]},7'h13};
      end
      5'b10010 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{{(4'b0000),_zz_62_[3 : 2]},_zz_62_[12]},_zz_62_[6 : 4]},(2'b00)},_zz_83_},(3'b010)},_zz_62_[11 : 7]},7'h03};
      end
      5'b10100 : begin
        IBusSimplePlugin_decompressor_decompressed = ((_zz_62_[12 : 2] == 11'h400) ? 32'h00100073 : ((_zz_62_[6 : 2] == 5'h0) ? {{{{12'h0,_zz_62_[11 : 7]},(3'b000)},(_zz_62_[12] ? _zz_82_ : _zz_81_)},7'h67} : {{{{{_zz_473_,_zz_474_},(_zz_475_ ? _zz_476_ : _zz_81_)},(3'b000)},_zz_62_[11 : 7]},7'h33}));
      end
      5'b10110 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_325_[11 : 5],_zz_62_[6 : 2]},_zz_83_},(3'b010)},_zz_326_[4 : 0]},7'h23};
      end
      default : begin
      end
    endcase
  end

  assign _zz_63_ = {(2'b01),_zz_62_[9 : 7]};
  assign _zz_64_ = {(2'b01),_zz_62_[4 : 2]};
  assign _zz_65_ = {{{{5'h0,_zz_62_[5]},_zz_62_[12 : 10]},_zz_62_[6]},(2'b00)};
  assign _zz_66_ = _zz_62_[12];
  always @ (*) begin
    _zz_67_[11] = _zz_66_;
    _zz_67_[10] = _zz_66_;
    _zz_67_[9] = _zz_66_;
    _zz_67_[8] = _zz_66_;
    _zz_67_[7] = _zz_66_;
    _zz_67_[6] = _zz_66_;
    _zz_67_[5] = _zz_66_;
    _zz_67_[4 : 0] = _zz_62_[6 : 2];
  end

  assign _zz_68_ = _zz_62_[12];
  always @ (*) begin
    _zz_69_[9] = _zz_68_;
    _zz_69_[8] = _zz_68_;
    _zz_69_[7] = _zz_68_;
    _zz_69_[6] = _zz_68_;
    _zz_69_[5] = _zz_68_;
    _zz_69_[4] = _zz_68_;
    _zz_69_[3] = _zz_68_;
    _zz_69_[2] = _zz_68_;
    _zz_69_[1] = _zz_68_;
    _zz_69_[0] = _zz_68_;
  end

  assign _zz_70_ = {{{{{{{{_zz_69_,_zz_62_[8]},_zz_62_[10 : 9]},_zz_62_[6]},_zz_62_[7]},_zz_62_[2]},_zz_62_[11]},_zz_62_[5 : 3]},(1'b0)};
  assign _zz_71_ = _zz_62_[12];
  always @ (*) begin
    _zz_72_[14] = _zz_71_;
    _zz_72_[13] = _zz_71_;
    _zz_72_[12] = _zz_71_;
    _zz_72_[11] = _zz_71_;
    _zz_72_[10] = _zz_71_;
    _zz_72_[9] = _zz_71_;
    _zz_72_[8] = _zz_71_;
    _zz_72_[7] = _zz_71_;
    _zz_72_[6] = _zz_71_;
    _zz_72_[5] = _zz_71_;
    _zz_72_[4] = _zz_71_;
    _zz_72_[3] = _zz_71_;
    _zz_72_[2] = _zz_71_;
    _zz_72_[1] = _zz_71_;
    _zz_72_[0] = _zz_71_;
  end

  assign _zz_73_ = _zz_62_[12];
  always @ (*) begin
    _zz_74_[2] = _zz_73_;
    _zz_74_[1] = _zz_73_;
    _zz_74_[0] = _zz_73_;
  end

  assign _zz_75_ = _zz_62_[12];
  always @ (*) begin
    _zz_76_[9] = _zz_75_;
    _zz_76_[8] = _zz_75_;
    _zz_76_[7] = _zz_75_;
    _zz_76_[6] = _zz_75_;
    _zz_76_[5] = _zz_75_;
    _zz_76_[4] = _zz_75_;
    _zz_76_[3] = _zz_75_;
    _zz_76_[2] = _zz_75_;
    _zz_76_[1] = _zz_75_;
    _zz_76_[0] = _zz_75_;
  end

  assign _zz_77_ = {{{{{{{{_zz_76_,_zz_62_[8]},_zz_62_[10 : 9]},_zz_62_[6]},_zz_62_[7]},_zz_62_[2]},_zz_62_[11]},_zz_62_[5 : 3]},(1'b0)};
  assign _zz_78_ = _zz_62_[12];
  always @ (*) begin
    _zz_79_[4] = _zz_78_;
    _zz_79_[3] = _zz_78_;
    _zz_79_[2] = _zz_78_;
    _zz_79_[1] = _zz_78_;
    _zz_79_[0] = _zz_78_;
  end

  assign _zz_80_ = {{{{{_zz_79_,_zz_62_[6 : 5]},_zz_62_[2]},_zz_62_[11 : 10]},_zz_62_[4 : 3]},(1'b0)};
  assign _zz_81_ = 5'h0;
  assign _zz_82_ = 5'h01;
  assign _zz_83_ = 5'h02;
  assign _zz_84_ = (_zz_62_[11 : 10] != (2'b11));
  always @ (*) begin
    case(_zz_282_)
      2'b00 : begin
        _zz_85_ = (3'b000);
      end
      2'b01 : begin
        _zz_85_ = (3'b100);
      end
      2'b10 : begin
        _zz_85_ = (3'b110);
      end
      default : begin
        _zz_85_ = (3'b111);
      end
    endcase
  end

  always @ (*) begin
    case(_zz_283_)
      2'b00 : begin
        _zz_86_ = (3'b101);
      end
      2'b01 : begin
        _zz_86_ = (3'b101);
      end
      2'b10 : begin
        _zz_86_ = (3'b111);
      end
      default : begin
        _zz_86_ = _zz_85_;
      end
    endcase
  end

  assign _zz_87_ = _zz_62_[12];
  always @ (*) begin
    _zz_88_[6] = _zz_87_;
    _zz_88_[5] = _zz_87_;
    _zz_88_[4] = _zz_87_;
    _zz_88_[3] = _zz_87_;
    _zz_88_[2] = _zz_87_;
    _zz_88_[1] = _zz_87_;
    _zz_88_[0] = _zz_87_;
  end

  assign IBusSimplePlugin_decompressor_output_valid = (IBusSimplePlugin_decompressor_input_valid && (! ((IBusSimplePlugin_decompressor_throw2Bytes && (! IBusSimplePlugin_decompressor_bufferValid)) && (! IBusSimplePlugin_decompressor_isInputHighRvc))));
  assign IBusSimplePlugin_decompressor_output_payload_pc = IBusSimplePlugin_decompressor_input_payload_pc;
  assign IBusSimplePlugin_decompressor_output_payload_isRvc = IBusSimplePlugin_decompressor_isRvc;
  assign IBusSimplePlugin_decompressor_output_payload_rsp_inst = (IBusSimplePlugin_decompressor_isRvc ? IBusSimplePlugin_decompressor_decompressed : IBusSimplePlugin_decompressor_raw);
  always @ (*) begin
    IBusSimplePlugin_decompressor_input_ready = (IBusSimplePlugin_decompressor_output_ready && (((! IBusSimplePlugin_iBusRsp_stages_1_input_valid) || IBusSimplePlugin_decompressor_flushNext) || ((! (IBusSimplePlugin_decompressor_bufferValid && IBusSimplePlugin_decompressor_isInputHighRvc)) && (! (((! IBusSimplePlugin_decompressor_unaligned) && IBusSimplePlugin_decompressor_isInputLowRvc) && IBusSimplePlugin_decompressor_isInputHighRvc)))));
    if(_zz_237_)begin
      IBusSimplePlugin_decompressor_input_ready = 1'b1;
    end
  end

  assign IBusSimplePlugin_decompressor_bufferFill = (((((! IBusSimplePlugin_decompressor_unaligned) && IBusSimplePlugin_decompressor_isInputLowRvc) && (! IBusSimplePlugin_decompressor_isInputHighRvc)) || (IBusSimplePlugin_decompressor_bufferValid && (! IBusSimplePlugin_decompressor_isInputHighRvc))) || ((IBusSimplePlugin_decompressor_throw2Bytes && (! IBusSimplePlugin_decompressor_isRvc)) && (! IBusSimplePlugin_decompressor_isInputHighRvc)));
  assign IBusSimplePlugin_decompressor_output_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_89_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_90_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_91_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_92_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_93_;
  assign IBusSimplePlugin_pcValids_0 = IBusSimplePlugin_injector_nextPcCalc_valids_0;
  assign IBusSimplePlugin_pcValids_1 = IBusSimplePlugin_injector_nextPcCalc_valids_1;
  assign IBusSimplePlugin_pcValids_2 = IBusSimplePlugin_injector_nextPcCalc_valids_2;
  assign IBusSimplePlugin_pcValids_3 = IBusSimplePlugin_injector_nextPcCalc_valids_3;
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  always @ (*) begin
    decode_arbitration_isValid = IBusSimplePlugin_injector_decodeInput_valid;
    case(_zz_172_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b011 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_51_ = (decode_PC + _zz_328_);
    if(IBusSimplePlugin_decodePc_predictionPcLoad_valid)begin
      _zz_51_ = IBusSimplePlugin_decodePc_predictionPcLoad_payload;
    end
  end

  assign IBusSimplePlugin_predictor_historyWriteDelayPatched_valid = IBusSimplePlugin_predictor_historyWrite_valid;
  assign IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_address = (IBusSimplePlugin_predictor_historyWrite_payload_address - 10'h001);
  assign IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_source = IBusSimplePlugin_predictor_historyWrite_payload_data_source;
  assign IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_branchWish = IBusSimplePlugin_predictor_historyWrite_payload_data_branchWish;
  assign IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_last2Bytes = IBusSimplePlugin_predictor_historyWrite_payload_data_last2Bytes;
  assign IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_target = IBusSimplePlugin_predictor_historyWrite_payload_data_target;
  assign _zz_94_ = (IBusSimplePlugin_iBusRsp_stages_0_input_payload >>> 2);
  assign _zz_95_ = _zz_196_;
  assign IBusSimplePlugin_predictor_buffer_line_source = _zz_95_[19 : 0];
  assign IBusSimplePlugin_predictor_buffer_line_branchWish = _zz_95_[21 : 20];
  assign IBusSimplePlugin_predictor_buffer_line_last2Bytes = _zz_330_[0];
  assign IBusSimplePlugin_predictor_buffer_line_target = _zz_95_[54 : 23];
  assign IBusSimplePlugin_predictor_buffer_hazard = (IBusSimplePlugin_predictor_writeLast_valid && (IBusSimplePlugin_predictor_writeLast_payload_address == _zz_332_));
  assign IBusSimplePlugin_predictor_hazard = (IBusSimplePlugin_predictor_buffer_hazard_regNextWhen || IBusSimplePlugin_predictor_buffer_pcCorrected);
  always @ (*) begin
    IBusSimplePlugin_predictor_hit = (IBusSimplePlugin_predictor_line_source == _zz_333_);
    if(((! IBusSimplePlugin_predictor_line_last2Bytes) && IBusSimplePlugin_iBusRsp_stages_1_input_payload[1]))begin
      IBusSimplePlugin_predictor_hit = 1'b0;
    end
  end

  assign IBusSimplePlugin_fetchPc_predictionPcLoad_valid = (((IBusSimplePlugin_predictor_line_branchWish[1] && IBusSimplePlugin_predictor_hit) && (! IBusSimplePlugin_predictor_hazard)) && IBusSimplePlugin_iBusRsp_stages_1_input_valid);
  assign IBusSimplePlugin_fetchPc_predictionPcLoad_payload = IBusSimplePlugin_predictor_line_target;
  assign IBusSimplePlugin_predictor_fetchContext_hazard = IBusSimplePlugin_predictor_hazard;
  assign IBusSimplePlugin_predictor_fetchContext_hit = IBusSimplePlugin_predictor_hit;
  assign IBusSimplePlugin_predictor_fetchContext_line_source = IBusSimplePlugin_predictor_line_source;
  assign IBusSimplePlugin_predictor_fetchContext_line_branchWish = IBusSimplePlugin_predictor_line_branchWish;
  assign IBusSimplePlugin_predictor_fetchContext_line_last2Bytes = IBusSimplePlugin_predictor_line_last2Bytes;
  assign IBusSimplePlugin_predictor_fetchContext_line_target = IBusSimplePlugin_predictor_line_target;
  assign IBusSimplePlugin_predictor_iBusRspContextOutput_hazard = IBusSimplePlugin_predictor_fetchContext_hazard;
  always @ (*) begin
    IBusSimplePlugin_predictor_iBusRspContextOutput_hit = IBusSimplePlugin_predictor_fetchContext_hit;
    if((IBusSimplePlugin_predictor_fetchContext_line_last2Bytes && (IBusSimplePlugin_decompressor_bufferValid || ((! IBusSimplePlugin_decompressor_throw2Bytes) && IBusSimplePlugin_decompressor_isInputLowRvc))))begin
      IBusSimplePlugin_predictor_iBusRspContextOutput_hit = 1'b0;
    end
  end

  assign IBusSimplePlugin_predictor_iBusRspContextOutput_line_source = IBusSimplePlugin_predictor_fetchContext_line_source;
  assign IBusSimplePlugin_predictor_iBusRspContextOutput_line_branchWish = IBusSimplePlugin_predictor_fetchContext_line_branchWish;
  assign IBusSimplePlugin_predictor_iBusRspContextOutput_line_last2Bytes = IBusSimplePlugin_predictor_fetchContext_line_last2Bytes;
  assign IBusSimplePlugin_predictor_iBusRspContextOutput_line_target = IBusSimplePlugin_predictor_fetchContext_line_target;
  assign IBusSimplePlugin_predictor_injectorContext_hazard = IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_hazard;
  assign IBusSimplePlugin_predictor_injectorContext_hit = IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_hit;
  assign IBusSimplePlugin_predictor_injectorContext_line_source = IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_source;
  assign IBusSimplePlugin_predictor_injectorContext_line_branchWish = IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_branchWish;
  assign IBusSimplePlugin_predictor_injectorContext_line_last2Bytes = IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_last2Bytes;
  assign IBusSimplePlugin_predictor_injectorContext_line_target = IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_target;
  assign IBusSimplePlugin_fetchPrediction_cmd_hadBranch = ((memory_PREDICTION_CONTEXT_hit && (! memory_PREDICTION_CONTEXT_hazard)) && memory_PREDICTION_CONTEXT_line_branchWish[1]);
  assign IBusSimplePlugin_fetchPrediction_cmd_targetPc = memory_PREDICTION_CONTEXT_line_target;
  always @ (*) begin
    IBusSimplePlugin_predictor_historyWrite_valid = 1'b0;
    if(IBusSimplePlugin_fetchPrediction_rsp_wasRight)begin
      IBusSimplePlugin_predictor_historyWrite_valid = memory_PREDICTION_CONTEXT_hit;
    end else begin
      if(memory_PREDICTION_CONTEXT_hit)begin
        IBusSimplePlugin_predictor_historyWrite_valid = 1'b1;
      end else begin
        IBusSimplePlugin_predictor_historyWrite_valid = 1'b1;
      end
    end
    if((memory_PREDICTION_CONTEXT_hazard || (! memory_arbitration_isFiring)))begin
      IBusSimplePlugin_predictor_historyWrite_valid = 1'b0;
    end
    if(IBusSimplePlugin_predictor_compressor_unalignedWordIssue)begin
      IBusSimplePlugin_predictor_historyWrite_valid = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_predictor_historyWrite_payload_address = IBusSimplePlugin_fetchPrediction_rsp_sourceLastWord[11 : 2];
    if(IBusSimplePlugin_predictor_compressor_unalignedWordIssue)begin
      IBusSimplePlugin_predictor_historyWrite_payload_address = _zz_344_[9:0];
    end
  end

  assign IBusSimplePlugin_predictor_historyWrite_payload_data_source = (IBusSimplePlugin_fetchPrediction_rsp_sourceLastWord >>> 12);
  assign IBusSimplePlugin_predictor_historyWrite_payload_data_target = IBusSimplePlugin_fetchPrediction_rsp_finalPc;
  assign IBusSimplePlugin_predictor_historyWrite_payload_data_last2Bytes = (memory_PC[1] && memory_IS_RVC);
  always @ (*) begin
    if(IBusSimplePlugin_fetchPrediction_rsp_wasRight)begin
      IBusSimplePlugin_predictor_historyWrite_payload_data_branchWish = (_zz_334_ - _zz_338_);
    end else begin
      if(memory_PREDICTION_CONTEXT_hit)begin
        IBusSimplePlugin_predictor_historyWrite_payload_data_branchWish = (_zz_339_ + _zz_343_);
      end else begin
        IBusSimplePlugin_predictor_historyWrite_payload_data_branchWish = (2'b10);
      end
    end
    if(IBusSimplePlugin_predictor_compressor_unalignedWordIssue)begin
      IBusSimplePlugin_predictor_historyWrite_payload_data_branchWish = (2'b00);
    end
  end

  assign IBusSimplePlugin_predictor_compressor_predictionBranch = ((IBusSimplePlugin_predictor_fetchContext_hit && (! IBusSimplePlugin_predictor_fetchContext_hazard)) && IBusSimplePlugin_predictor_fetchContext_line_branchWish[1]);
  assign IBusSimplePlugin_predictor_compressor_unalignedWordIssue = (((IBusSimplePlugin_iBusRsp_output_valid && IBusSimplePlugin_predictor_compressor_predictionBranch) && IBusSimplePlugin_predictor_fetchContext_line_last2Bytes) && (IBusSimplePlugin_decompressor_unaligned ? (! IBusSimplePlugin_decompressor_isInputHighRvc) : (IBusSimplePlugin_decompressor_isInputLowRvc && (! IBusSimplePlugin_decompressor_isInputHighRvc))));
  assign IBusSimplePlugin_decodePc_predictionPcLoad_valid = (((IBusSimplePlugin_predictor_injectorContext_line_branchWish[1] && IBusSimplePlugin_predictor_injectorContext_hit) && (! IBusSimplePlugin_predictor_injectorContext_hazard)) && (IBusSimplePlugin_injector_decodeInput_valid && IBusSimplePlugin_injector_decodeInput_ready));
  assign IBusSimplePlugin_decodePc_predictionPcLoad_payload = IBusSimplePlugin_predictor_injectorContext_line_target;
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pending_next = (_zz_345_ - _zz_349_);
  assign IBusSimplePlugin_cmdFork_canEmit = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && (IBusSimplePlugin_pending_value != (3'b111)));
  always @ (*) begin
    IBusSimplePlugin_cmd_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && IBusSimplePlugin_cmdFork_canEmit);
    if(IBusSimplePlugin_iBusRsp_stages_0_input_valid)begin
      if(IBusSimplePlugin_mmuBus_rsp_refilling)begin
        IBusSimplePlugin_cmd_valid = 1'b0;
      end
      if(IBusSimplePlugin_mmuBus_rsp_exception)begin
        IBusSimplePlugin_cmd_valid = 1'b0;
      end
    end
  end

  assign IBusSimplePlugin_pending_inc = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign IBusSimplePlugin_mmuBus_cmd_isValid = IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  assign IBusSimplePlugin_mmuBus_cmd_virtualAddress = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  assign IBusSimplePlugin_mmuBus_cmd_bypassTranslation = 1'b0;
  assign IBusSimplePlugin_mmuBus_end = ((IBusSimplePlugin_iBusRsp_stages_0_output_valid && IBusSimplePlugin_iBusRsp_stages_0_output_ready) || IBusSimplePlugin_externalFlush);
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 2],(2'b00)};
  assign IBusSimplePlugin_rspJoin_rspBuffer_flush = ((IBusSimplePlugin_rspJoin_rspBuffer_discardCounter != (3'b000)) || IBusSimplePlugin_iBusRsp_flush);
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_valid = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter == (3'b000)));
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  assign _zz_194_ = (IBusSimplePlugin_rspJoin_rspBuffer_output_ready || IBusSimplePlugin_rspJoin_rspBuffer_flush);
  assign IBusSimplePlugin_pending_dec = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && _zz_194_);
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBuffer_output_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_exceptionDetected = 1'b0;
    if(_zz_238_)begin
      IBusSimplePlugin_rspJoin_exceptionDetected = 1'b1;
    end
    if(_zz_239_)begin
      IBusSimplePlugin_rspJoin_exceptionDetected = 1'b1;
    end
  end

  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_1_output_valid && IBusSimplePlugin_rspJoin_rspBuffer_output_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_96_ = (! IBusSimplePlugin_rspJoin_exceptionDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_output_ready && _zz_96_);
  assign IBusSimplePlugin_iBusRsp_output_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_96_);
  assign IBusSimplePlugin_iBusRsp_output_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_output_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_output_payload_rsp_inst = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_output_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  always @ (*) begin
    IBusSimplePlugin_decodeExceptionPort_payload_code = (4'bxxxx);
    if(_zz_238_)begin
      IBusSimplePlugin_decodeExceptionPort_payload_code = (4'b0001);
    end
    if(_zz_239_)begin
      IBusSimplePlugin_decodeExceptionPort_payload_code = (4'b1100);
    end
  end

  assign IBusSimplePlugin_decodeExceptionPort_payload_badAddr = {IBusSimplePlugin_rspJoin_join_payload_pc[31 : 2],(2'b00)};
  assign IBusSimplePlugin_decodeExceptionPort_valid = (IBusSimplePlugin_rspJoin_exceptionDetected && IBusSimplePlugin_iBusRsp_readyForError);
  assign _zz_97_ = 1'b0;
  always @ (*) begin
    execute_DBusSimplePlugin_skipCmd = 1'b0;
    if(execute_ALIGNEMENT_FAULT)begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
    if((execute_MMU_FAULT || execute_MMU_RSP_refilling))begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
    if(((execute_MEMORY_STORE && execute_MEMORY_ATOMIC) && (! execute_ATOMIC_HIT)))begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
  end

  always @ (*) begin
    dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_97_));
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
        dBus_cmd_valid = 1'b1;
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dBus_cmd_payload_wr = execute_MEMORY_STORE;
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
        dBus_cmd_payload_wr = MmuPlugin_dBusAccess_cmd_payload_write;
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
        dBus_cmd_payload_size = MmuPlugin_dBusAccess_cmd_payload_size;
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_98_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_98_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_98_ = execute_RS2[31 : 0];
      end
    endcase
  end

  always @ (*) begin
    dBus_cmd_payload_data = _zz_98_;
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
        dBus_cmd_payload_data = MmuPlugin_dBusAccess_cmd_payload_data;
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_99_ = (4'b0001);
      end
      2'b01 : begin
        _zz_99_ = (4'b0011);
      end
      default : begin
        _zz_99_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_99_ <<< dBus_cmd_payload_address[1 : 0]);
  assign DBusSimplePlugin_mmuBus_cmd_isValid = (execute_arbitration_isValid && execute_MEMORY_ENABLE);
  assign DBusSimplePlugin_mmuBus_cmd_virtualAddress = execute_SRC_ADD;
  assign DBusSimplePlugin_mmuBus_cmd_bypassTranslation = 1'b0;
  assign DBusSimplePlugin_mmuBus_end = ((! execute_arbitration_isStuck) || execute_arbitration_removeIt);
  always @ (*) begin
    dBus_cmd_payload_address = DBusSimplePlugin_mmuBus_rsp_physicalAddress;
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
        dBus_cmd_payload_address = MmuPlugin_dBusAccess_cmd_payload_address;
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    DBusSimplePlugin_memoryExceptionPort_valid = 1'b0;
    if(_zz_240_)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b1;
    end
    if(memory_ALIGNEMENT_FAULT)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b1;
    end
    if(memory_MMU_RSP_refilling)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b0;
    end else begin
      if(memory_MMU_FAULT)begin
        DBusSimplePlugin_memoryExceptionPort_valid = 1'b1;
      end
    end
    if(_zz_241_)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b0;
    end
  end

  always @ (*) begin
    DBusSimplePlugin_memoryExceptionPort_payload_code = (4'bxxxx);
    if(_zz_240_)begin
      DBusSimplePlugin_memoryExceptionPort_payload_code = (4'b0101);
    end
    if(memory_ALIGNEMENT_FAULT)begin
      DBusSimplePlugin_memoryExceptionPort_payload_code = {1'd0, _zz_354_};
    end
    if(! memory_MMU_RSP_refilling) begin
      if(memory_MMU_FAULT)begin
        DBusSimplePlugin_memoryExceptionPort_payload_code = (memory_MEMORY_STORE ? (4'b1111) : (4'b1101));
      end
    end
  end

  assign DBusSimplePlugin_memoryExceptionPort_payload_badAddr = memory_REGFILE_WRITE_DATA;
  always @ (*) begin
    DBusSimplePlugin_redoBranch_valid = 1'b0;
    if(memory_MMU_RSP_refilling)begin
      DBusSimplePlugin_redoBranch_valid = 1'b1;
    end
    if(_zz_241_)begin
      DBusSimplePlugin_redoBranch_valid = 1'b0;
    end
  end

  assign DBusSimplePlugin_redoBranch_payload = memory_PC;
  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_100_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_101_[31] = _zz_100_;
    _zz_101_[30] = _zz_100_;
    _zz_101_[29] = _zz_100_;
    _zz_101_[28] = _zz_100_;
    _zz_101_[27] = _zz_100_;
    _zz_101_[26] = _zz_100_;
    _zz_101_[25] = _zz_100_;
    _zz_101_[24] = _zz_100_;
    _zz_101_[23] = _zz_100_;
    _zz_101_[22] = _zz_100_;
    _zz_101_[21] = _zz_100_;
    _zz_101_[20] = _zz_100_;
    _zz_101_[19] = _zz_100_;
    _zz_101_[18] = _zz_100_;
    _zz_101_[17] = _zz_100_;
    _zz_101_[16] = _zz_100_;
    _zz_101_[15] = _zz_100_;
    _zz_101_[14] = _zz_100_;
    _zz_101_[13] = _zz_100_;
    _zz_101_[12] = _zz_100_;
    _zz_101_[11] = _zz_100_;
    _zz_101_[10] = _zz_100_;
    _zz_101_[9] = _zz_100_;
    _zz_101_[8] = _zz_100_;
    _zz_101_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_102_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_103_[31] = _zz_102_;
    _zz_103_[30] = _zz_102_;
    _zz_103_[29] = _zz_102_;
    _zz_103_[28] = _zz_102_;
    _zz_103_[27] = _zz_102_;
    _zz_103_[26] = _zz_102_;
    _zz_103_[25] = _zz_102_;
    _zz_103_[24] = _zz_102_;
    _zz_103_[23] = _zz_102_;
    _zz_103_[22] = _zz_102_;
    _zz_103_[21] = _zz_102_;
    _zz_103_[20] = _zz_102_;
    _zz_103_[19] = _zz_102_;
    _zz_103_[18] = _zz_102_;
    _zz_103_[17] = _zz_102_;
    _zz_103_[16] = _zz_102_;
    _zz_103_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_284_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_101_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_103_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  always @ (*) begin
    MmuPlugin_dBusAccess_cmd_ready = 1'b0;
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
        if(dBus_cmd_ready)begin
          MmuPlugin_dBusAccess_cmd_ready = 1'b1;
        end
      end
      2'b10 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    MmuPlugin_dBusAccess_rsp_valid = 1'b0;
    case(_zz_104_)
      2'b00 : begin
      end
      2'b01 : begin
      end
      2'b10 : begin
        if(dBus_rsp_ready)begin
          MmuPlugin_dBusAccess_rsp_valid = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign MmuPlugin_dBusAccess_rsp_payload_data = dBus_rsp_data;
  assign MmuPlugin_dBusAccess_rsp_payload_error = dBus_rsp_error;
  assign MmuPlugin_dBusAccess_rsp_payload_redo = 1'b0;
  assign _zz_106_ = ((decode_INSTRUCTION & 32'h00000004) == 32'h00000004);
  assign _zz_107_ = ((decode_INSTRUCTION & 32'h00001000) == 32'h0);
  assign _zz_108_ = ((decode_INSTRUCTION & 32'h00002050) == 32'h00002000);
  assign _zz_109_ = ((decode_INSTRUCTION & 32'h00004050) == 32'h00004050);
  assign _zz_110_ = ((decode_INSTRUCTION & 32'h00000048) == 32'h00000048);
  assign _zz_111_ = ((decode_INSTRUCTION & 32'h0000000c) == 32'h00000004);
  assign _zz_105_ = {({_zz_111_,{_zz_477_,{_zz_478_,_zz_479_}}} != 5'h0),{((_zz_480_ == _zz_481_) != (1'b0)),{(_zz_482_ != (1'b0)),{(_zz_483_ != _zz_484_),{_zz_485_,{_zz_486_,_zz_487_}}}}}};
  assign _zz_112_ = _zz_105_[4 : 3];
  assign _zz_46_ = _zz_112_;
  assign _zz_113_ = _zz_105_[6 : 5];
  assign _zz_45_ = _zz_113_;
  assign _zz_114_ = _zz_105_[10 : 9];
  assign _zz_44_ = _zz_114_;
  assign _zz_115_ = _zz_105_[16 : 15];
  assign _zz_43_ = _zz_115_;
  assign _zz_116_ = _zz_105_[22 : 21];
  assign _zz_42_ = _zz_116_;
  assign _zz_117_ = _zz_105_[25 : 24];
  assign _zz_41_ = _zz_117_;
  assign _zz_118_ = _zz_105_[31 : 30];
  assign _zz_40_ = _zz_118_;
  assign decodeExceptionPort_valid = (decode_arbitration_isValid && (! decode_LEGAL_INSTRUCTION));
  assign decodeExceptionPort_payload_code = (4'b0010);
  assign decodeExceptionPort_payload_badAddr = decode_INSTRUCTION;
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_197_;
  assign decode_RegFilePlugin_rs2Data = _zz_198_;
  always @ (*) begin
    lastStageRegFileWrite_valid = (_zz_38_ && writeBack_arbitration_isFiring);
    if(_zz_119_)begin
      lastStageRegFileWrite_valid = 1'b1;
    end
  end

  assign lastStageRegFileWrite_payload_address = _zz_37_[11 : 7];
  assign lastStageRegFileWrite_payload_data = _zz_47_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_120_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_120_ = {31'd0, _zz_356_};
      end
      default : begin
        _zz_120_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_121_ = _zz_33_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_121_ = {29'd0, _zz_357_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_121_ = {decode_INSTRUCTION[31 : 12],12'h0};
      end
      default : begin
        _zz_121_ = {27'd0, _zz_358_};
      end
    endcase
  end

  assign _zz_122_ = _zz_359_[11];
  always @ (*) begin
    _zz_123_[19] = _zz_122_;
    _zz_123_[18] = _zz_122_;
    _zz_123_[17] = _zz_122_;
    _zz_123_[16] = _zz_122_;
    _zz_123_[15] = _zz_122_;
    _zz_123_[14] = _zz_122_;
    _zz_123_[13] = _zz_122_;
    _zz_123_[12] = _zz_122_;
    _zz_123_[11] = _zz_122_;
    _zz_123_[10] = _zz_122_;
    _zz_123_[9] = _zz_122_;
    _zz_123_[8] = _zz_122_;
    _zz_123_[7] = _zz_122_;
    _zz_123_[6] = _zz_122_;
    _zz_123_[5] = _zz_122_;
    _zz_123_[4] = _zz_122_;
    _zz_123_[3] = _zz_122_;
    _zz_123_[2] = _zz_122_;
    _zz_123_[1] = _zz_122_;
    _zz_123_[0] = _zz_122_;
  end

  assign _zz_124_ = _zz_360_[11];
  always @ (*) begin
    _zz_125_[19] = _zz_124_;
    _zz_125_[18] = _zz_124_;
    _zz_125_[17] = _zz_124_;
    _zz_125_[16] = _zz_124_;
    _zz_125_[15] = _zz_124_;
    _zz_125_[14] = _zz_124_;
    _zz_125_[13] = _zz_124_;
    _zz_125_[12] = _zz_124_;
    _zz_125_[11] = _zz_124_;
    _zz_125_[10] = _zz_124_;
    _zz_125_[9] = _zz_124_;
    _zz_125_[8] = _zz_124_;
    _zz_125_[7] = _zz_124_;
    _zz_125_[6] = _zz_124_;
    _zz_125_[5] = _zz_124_;
    _zz_125_[4] = _zz_124_;
    _zz_125_[3] = _zz_124_;
    _zz_125_[2] = _zz_124_;
    _zz_125_[1] = _zz_124_;
    _zz_125_[0] = _zz_124_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_126_ = _zz_31_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_126_ = {_zz_123_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_126_ = {_zz_125_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_126_ = _zz_30_;
      end
    endcase
  end

  always @ (*) begin
    execute_SrcPlugin_addSub = _zz_361_;
    if(execute_SRC2_FORCE_ZERO)begin
      execute_SrcPlugin_addSub = execute_SRC1;
    end
  end

  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign execute_FullBarrelShifterPlugin_amplitude = execute_SRC2[4 : 0];
  always @ (*) begin
    _zz_127_[0] = execute_SRC1[31];
    _zz_127_[1] = execute_SRC1[30];
    _zz_127_[2] = execute_SRC1[29];
    _zz_127_[3] = execute_SRC1[28];
    _zz_127_[4] = execute_SRC1[27];
    _zz_127_[5] = execute_SRC1[26];
    _zz_127_[6] = execute_SRC1[25];
    _zz_127_[7] = execute_SRC1[24];
    _zz_127_[8] = execute_SRC1[23];
    _zz_127_[9] = execute_SRC1[22];
    _zz_127_[10] = execute_SRC1[21];
    _zz_127_[11] = execute_SRC1[20];
    _zz_127_[12] = execute_SRC1[19];
    _zz_127_[13] = execute_SRC1[18];
    _zz_127_[14] = execute_SRC1[17];
    _zz_127_[15] = execute_SRC1[16];
    _zz_127_[16] = execute_SRC1[15];
    _zz_127_[17] = execute_SRC1[14];
    _zz_127_[18] = execute_SRC1[13];
    _zz_127_[19] = execute_SRC1[12];
    _zz_127_[20] = execute_SRC1[11];
    _zz_127_[21] = execute_SRC1[10];
    _zz_127_[22] = execute_SRC1[9];
    _zz_127_[23] = execute_SRC1[8];
    _zz_127_[24] = execute_SRC1[7];
    _zz_127_[25] = execute_SRC1[6];
    _zz_127_[26] = execute_SRC1[5];
    _zz_127_[27] = execute_SRC1[4];
    _zz_127_[28] = execute_SRC1[3];
    _zz_127_[29] = execute_SRC1[2];
    _zz_127_[30] = execute_SRC1[1];
    _zz_127_[31] = execute_SRC1[0];
  end

  assign execute_FullBarrelShifterPlugin_reversed = ((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SLL_1) ? _zz_127_ : execute_SRC1);
  always @ (*) begin
    _zz_128_[0] = memory_SHIFT_RIGHT[31];
    _zz_128_[1] = memory_SHIFT_RIGHT[30];
    _zz_128_[2] = memory_SHIFT_RIGHT[29];
    _zz_128_[3] = memory_SHIFT_RIGHT[28];
    _zz_128_[4] = memory_SHIFT_RIGHT[27];
    _zz_128_[5] = memory_SHIFT_RIGHT[26];
    _zz_128_[6] = memory_SHIFT_RIGHT[25];
    _zz_128_[7] = memory_SHIFT_RIGHT[24];
    _zz_128_[8] = memory_SHIFT_RIGHT[23];
    _zz_128_[9] = memory_SHIFT_RIGHT[22];
    _zz_128_[10] = memory_SHIFT_RIGHT[21];
    _zz_128_[11] = memory_SHIFT_RIGHT[20];
    _zz_128_[12] = memory_SHIFT_RIGHT[19];
    _zz_128_[13] = memory_SHIFT_RIGHT[18];
    _zz_128_[14] = memory_SHIFT_RIGHT[17];
    _zz_128_[15] = memory_SHIFT_RIGHT[16];
    _zz_128_[16] = memory_SHIFT_RIGHT[15];
    _zz_128_[17] = memory_SHIFT_RIGHT[14];
    _zz_128_[18] = memory_SHIFT_RIGHT[13];
    _zz_128_[19] = memory_SHIFT_RIGHT[12];
    _zz_128_[20] = memory_SHIFT_RIGHT[11];
    _zz_128_[21] = memory_SHIFT_RIGHT[10];
    _zz_128_[22] = memory_SHIFT_RIGHT[9];
    _zz_128_[23] = memory_SHIFT_RIGHT[8];
    _zz_128_[24] = memory_SHIFT_RIGHT[7];
    _zz_128_[25] = memory_SHIFT_RIGHT[6];
    _zz_128_[26] = memory_SHIFT_RIGHT[5];
    _zz_128_[27] = memory_SHIFT_RIGHT[4];
    _zz_128_[28] = memory_SHIFT_RIGHT[3];
    _zz_128_[29] = memory_SHIFT_RIGHT[2];
    _zz_128_[30] = memory_SHIFT_RIGHT[1];
    _zz_128_[31] = memory_SHIFT_RIGHT[0];
  end

  always @ (*) begin
    _zz_129_ = 1'b0;
    if(_zz_242_)begin
      if(_zz_243_)begin
        if(_zz_134_)begin
          _zz_129_ = 1'b1;
        end
      end
    end
    if(_zz_244_)begin
      if(_zz_245_)begin
        if(_zz_136_)begin
          _zz_129_ = 1'b1;
        end
      end
    end
    if(_zz_246_)begin
      if(_zz_247_)begin
        if(_zz_138_)begin
          _zz_129_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_129_ = 1'b0;
    end
  end

  always @ (*) begin
    _zz_130_ = 1'b0;
    if(_zz_242_)begin
      if(_zz_243_)begin
        if(_zz_135_)begin
          _zz_130_ = 1'b1;
        end
      end
    end
    if(_zz_244_)begin
      if(_zz_245_)begin
        if(_zz_137_)begin
          _zz_130_ = 1'b1;
        end
      end
    end
    if(_zz_246_)begin
      if(_zz_247_)begin
        if(_zz_139_)begin
          _zz_130_ = 1'b1;
        end
      end
    end
    if((! decode_RS2_USE))begin
      _zz_130_ = 1'b0;
    end
  end

  assign _zz_134_ = (writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_135_ = (writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign _zz_136_ = (memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_137_ = (memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign _zz_138_ = (execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_139_ = (execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign execute_MulPlugin_a = execute_RS1;
  assign execute_MulPlugin_b = execute_RS2;
  always @ (*) begin
    case(_zz_248_)
      2'b01 : begin
        execute_MulPlugin_aSigned = 1'b1;
      end
      2'b10 : begin
        execute_MulPlugin_aSigned = 1'b1;
      end
      default : begin
        execute_MulPlugin_aSigned = 1'b0;
      end
    endcase
  end

  always @ (*) begin
    case(_zz_248_)
      2'b01 : begin
        execute_MulPlugin_bSigned = 1'b1;
      end
      2'b10 : begin
        execute_MulPlugin_bSigned = 1'b0;
      end
      default : begin
        execute_MulPlugin_bSigned = 1'b0;
      end
    endcase
  end

  assign execute_MulPlugin_aULow = execute_MulPlugin_a[15 : 0];
  assign execute_MulPlugin_bULow = execute_MulPlugin_b[15 : 0];
  assign execute_MulPlugin_aSLow = {1'b0,execute_MulPlugin_a[15 : 0]};
  assign execute_MulPlugin_bSLow = {1'b0,execute_MulPlugin_b[15 : 0]};
  assign execute_MulPlugin_aHigh = {(execute_MulPlugin_aSigned && execute_MulPlugin_a[31]),execute_MulPlugin_a[31 : 16]};
  assign execute_MulPlugin_bHigh = {(execute_MulPlugin_bSigned && execute_MulPlugin_b[31]),execute_MulPlugin_b[31 : 16]};
  assign writeBack_MulPlugin_result = ($signed(_zz_368_) + $signed(_zz_369_));
  assign memory_MulDivIterativePlugin_frontendOk = 1'b1;
  always @ (*) begin
    memory_MulDivIterativePlugin_div_counter_willIncrement = 1'b0;
    if(_zz_227_)begin
      if(_zz_249_)begin
        memory_MulDivIterativePlugin_div_counter_willIncrement = 1'b1;
      end
    end
  end

  always @ (*) begin
    memory_MulDivIterativePlugin_div_counter_willClear = 1'b0;
    if(_zz_250_)begin
      memory_MulDivIterativePlugin_div_counter_willClear = 1'b1;
    end
  end

  assign memory_MulDivIterativePlugin_div_counter_willOverflowIfInc = (memory_MulDivIterativePlugin_div_counter_value == 6'h21);
  assign memory_MulDivIterativePlugin_div_counter_willOverflow = (memory_MulDivIterativePlugin_div_counter_willOverflowIfInc && memory_MulDivIterativePlugin_div_counter_willIncrement);
  always @ (*) begin
    if(memory_MulDivIterativePlugin_div_counter_willOverflow)begin
      memory_MulDivIterativePlugin_div_counter_valueNext = 6'h0;
    end else begin
      memory_MulDivIterativePlugin_div_counter_valueNext = (memory_MulDivIterativePlugin_div_counter_value + _zz_373_);
    end
    if(memory_MulDivIterativePlugin_div_counter_willClear)begin
      memory_MulDivIterativePlugin_div_counter_valueNext = 6'h0;
    end
  end

  assign _zz_140_ = memory_MulDivIterativePlugin_rs1[31 : 0];
  assign memory_MulDivIterativePlugin_div_stage_0_remainderShifted = {memory_MulDivIterativePlugin_accumulator[31 : 0],_zz_140_[31]};
  assign memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator = (memory_MulDivIterativePlugin_div_stage_0_remainderShifted - _zz_374_);
  assign memory_MulDivIterativePlugin_div_stage_0_outRemainder = ((! memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator[32]) ? _zz_375_ : _zz_376_);
  assign memory_MulDivIterativePlugin_div_stage_0_outNumerator = _zz_377_[31:0];
  assign _zz_141_ = (memory_INSTRUCTION[13] ? memory_MulDivIterativePlugin_accumulator[31 : 0] : memory_MulDivIterativePlugin_rs1[31 : 0]);
  assign _zz_142_ = (execute_RS2[31] && execute_IS_RS2_SIGNED);
  assign _zz_143_ = (1'b0 || ((execute_IS_DIV && execute_RS1[31]) && execute_IS_RS1_SIGNED));
  always @ (*) begin
    _zz_144_[32] = (execute_IS_RS1_SIGNED && execute_RS1[31]);
    _zz_144_[31 : 0] = execute_RS1;
  end

  always @ (*) begin
    CsrPlugin_privilege = _zz_145_;
    if(CsrPlugin_forceMachineWire)begin
      CsrPlugin_privilege = (2'b11);
    end
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = 26'h0;
  assign CsrPlugin_sip_SEIP_OR = (CsrPlugin_sip_SEIP_SOFT || CsrPlugin_sip_SEIP_INPUT);
  always @ (*) begin
    CsrPlugin_redoInterface_valid = 1'b0;
    if(execute_CsrPlugin_csr_384)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_redoInterface_valid = 1'b1;
      end
    end
  end

  assign CsrPlugin_redoInterface_payload = decode_PC;
  assign _zz_146_ = (CsrPlugin_sip_STIP && CsrPlugin_sie_STIE);
  assign _zz_147_ = (CsrPlugin_sip_SSIP && CsrPlugin_sie_SSIE);
  assign _zz_148_ = (CsrPlugin_sip_SEIP_OR && CsrPlugin_sie_SEIE);
  assign _zz_149_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_150_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_151_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b11);
    case(CsrPlugin_exceptionPortCtrl_exceptionContext_code)
      4'b1000 : begin
        if(((1'b1 && CsrPlugin_medeleg_EU) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0010 : begin
        if(((1'b1 && CsrPlugin_medeleg_II) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0101 : begin
        if(((1'b1 && CsrPlugin_medeleg_LAF) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b1101 : begin
        if(((1'b1 && CsrPlugin_medeleg_LPF) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0100 : begin
        if(((1'b1 && CsrPlugin_medeleg_LAM) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0111 : begin
        if(((1'b1 && CsrPlugin_medeleg_SAF) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0001 : begin
        if(((1'b1 && CsrPlugin_medeleg_IAF) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b1001 : begin
        if(((1'b1 && CsrPlugin_medeleg_ES) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b1100 : begin
        if(((1'b1 && CsrPlugin_medeleg_IPF) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b1111 : begin
        if(((1'b1 && CsrPlugin_medeleg_SPF) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0110 : begin
        if(((1'b1 && CsrPlugin_medeleg_SAM) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      4'b0000 : begin
        if(((1'b1 && CsrPlugin_medeleg_IAM) && (! 1'b0)))begin
          CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b01);
        end
      end
      default : begin
      end
    endcase
  end

  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege = ((CsrPlugin_privilege < CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped) ? CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped : CsrPlugin_privilege);
  assign _zz_152_ = {decodeExceptionPort_valid,IBusSimplePlugin_decodeExceptionPort_valid};
  assign _zz_153_ = _zz_387_[0];
  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_decode = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
    if(_zz_228_)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_decode = 1'b1;
    end
    if(decode_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_decode = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_execute = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_memory = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
    if(DBusSimplePlugin_memoryExceptionPort_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b1;
    end
    if(memory_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
    if(writeBack_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = 1'b0;
    end
  end

  assign CsrPlugin_exceptionPendings_0 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  assign CsrPlugin_exceptionPendings_1 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  assign CsrPlugin_exceptionPendings_2 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  assign CsrPlugin_exceptionPendings_3 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  assign CsrPlugin_exception = (CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack && CsrPlugin_allowException);
  assign CsrPlugin_pipelineLiberator_active = ((CsrPlugin_interrupt_valid && CsrPlugin_allowInterrupts) && decode_arbitration_isValid);
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = CsrPlugin_pipelineLiberator_pcValids_2;
    if(({CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack,{CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory,CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute}} != (3'b000)))begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = ((CsrPlugin_interrupt_valid && CsrPlugin_pipelineLiberator_done) && CsrPlugin_allowInterrupts);
  always @ (*) begin
    CsrPlugin_targetPrivilege = CsrPlugin_interrupt_targetPrivilege;
    if(CsrPlugin_hadException)begin
      CsrPlugin_targetPrivilege = CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
    end
  end

  always @ (*) begin
    CsrPlugin_trapCause = CsrPlugin_interrupt_code;
    if(CsrPlugin_hadException)begin
      CsrPlugin_trapCause = CsrPlugin_exceptionPortCtrl_exceptionContext_code;
    end
  end

  always @ (*) begin
    CsrPlugin_xtvec_mode = (2'bxx);
    case(CsrPlugin_targetPrivilege)
      2'b01 : begin
        CsrPlugin_xtvec_mode = CsrPlugin_stvec_mode;
      end
      2'b11 : begin
        CsrPlugin_xtvec_mode = CsrPlugin_mtvec_mode;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    CsrPlugin_xtvec_base = 30'h0;
    case(CsrPlugin_targetPrivilege)
      2'b01 : begin
        CsrPlugin_xtvec_base = CsrPlugin_stvec_base;
      end
      2'b11 : begin
        CsrPlugin_xtvec_base = CsrPlugin_mtvec_base;
      end
      default : begin
      end
    endcase
  end

  assign contextSwitching = CsrPlugin_jumpInterface_valid;
  assign execute_CsrPlugin_blockedBySideEffects = ({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00));
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = 1'b1;
    if(execute_CsrPlugin_csr_3857)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_3858)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_3859)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_3860)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_768)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_836)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_772)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_773)begin
      if(execute_CSR_WRITE_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_833)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_832)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_834)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_835)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_770)begin
      if(execute_CSR_WRITE_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_771)begin
      if(execute_CSR_WRITE_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(execute_CsrPlugin_csr_256)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_324)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_260)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_261)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_321)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_320)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_322)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_323)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_384)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(_zz_251_)begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
    if(((! execute_arbitration_isValid) || (! execute_IS_CSR)))begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((CsrPlugin_privilege < execute_INSTRUCTION[29 : 28]))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_valid = 1'b0;
    if(_zz_252_)begin
      CsrPlugin_selfException_valid = 1'b1;
    end
    if(_zz_253_)begin
      CsrPlugin_selfException_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_payload_code = (4'bxxxx);
    if(_zz_252_)begin
      CsrPlugin_selfException_payload_code = (4'b0010);
    end
    if(_zz_253_)begin
      case(CsrPlugin_privilege)
        2'b00 : begin
          CsrPlugin_selfException_payload_code = (4'b1000);
        end
        2'b01 : begin
          CsrPlugin_selfException_payload_code = (4'b1001);
        end
        default : begin
          CsrPlugin_selfException_payload_code = (4'b1011);
        end
      endcase
    end
  end

  assign CsrPlugin_selfException_payload_badAddr = execute_INSTRUCTION;
  always @ (*) begin
    execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
    if(_zz_251_)begin
      execute_CsrPlugin_writeInstruction = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
    if(_zz_251_)begin
      execute_CsrPlugin_readInstruction = 1'b0;
    end
  end

  assign execute_CsrPlugin_writeEnable = (execute_CsrPlugin_writeInstruction && (! execute_arbitration_isStuck));
  assign execute_CsrPlugin_readEnable = (execute_CsrPlugin_readInstruction && (! execute_arbitration_isStuck));
  always @ (*) begin
    execute_CsrPlugin_readToWriteData = execute_CsrPlugin_readData;
    if(execute_CsrPlugin_csr_836)begin
      execute_CsrPlugin_readToWriteData[9 : 9] = CsrPlugin_sip_SEIP_SOFT;
    end
    if(execute_CsrPlugin_csr_324)begin
      execute_CsrPlugin_readToWriteData[9 : 9] = CsrPlugin_sip_SEIP_SOFT;
    end
  end

  always @ (*) begin
    case(_zz_286_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readToWriteData & (~ execute_SRC1)) : (execute_CsrPlugin_readToWriteData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  always @ (*) begin
    debug_bus_cmd_ready = 1'b1;
    if(debug_bus_cmd_valid)begin
      case(_zz_254_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            debug_bus_cmd_ready = IBusSimplePlugin_injectionPort_ready;
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    debug_bus_rsp_data = DebugPlugin_busReadDataReg;
    if((! _zz_154_))begin
      debug_bus_rsp_data[0] = DebugPlugin_resetIt;
      debug_bus_rsp_data[1] = DebugPlugin_haltIt;
      debug_bus_rsp_data[2] = DebugPlugin_isPipBusy;
      debug_bus_rsp_data[3] = DebugPlugin_haltedByBreak;
      debug_bus_rsp_data[4] = DebugPlugin_stepIt;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_injectionPort_valid = 1'b0;
    if(debug_bus_cmd_valid)begin
      case(_zz_254_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            IBusSimplePlugin_injectionPort_valid = 1'b1;
          end
        end
        default : begin
        end
      endcase
    end
  end

  assign IBusSimplePlugin_injectionPort_payload = debug_bus_cmd_payload_data;
  assign DebugPlugin_allowEBreak = (CsrPlugin_privilege == (2'b11));
  assign debug_resetOut = DebugPlugin_resetIt_regNext;
  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_156_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_156_ == (3'b000))) begin
        _zz_157_ = execute_BranchPlugin_eq;
    end else if((_zz_156_ == (3'b001))) begin
        _zz_157_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_156_ & (3'b101)) == (3'b101)))) begin
        _zz_157_ = (! execute_SRC_LESS);
    end else begin
        _zz_157_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_158_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_158_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_158_ = 1'b1;
      end
      default : begin
        _zz_158_ = _zz_157_;
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_159_ = _zz_389_[19];
  always @ (*) begin
    _zz_160_[10] = _zz_159_;
    _zz_160_[9] = _zz_159_;
    _zz_160_[8] = _zz_159_;
    _zz_160_[7] = _zz_159_;
    _zz_160_[6] = _zz_159_;
    _zz_160_[5] = _zz_159_;
    _zz_160_[4] = _zz_159_;
    _zz_160_[3] = _zz_159_;
    _zz_160_[2] = _zz_159_;
    _zz_160_[1] = _zz_159_;
    _zz_160_[0] = _zz_159_;
  end

  assign _zz_161_ = _zz_390_[11];
  always @ (*) begin
    _zz_162_[19] = _zz_161_;
    _zz_162_[18] = _zz_161_;
    _zz_162_[17] = _zz_161_;
    _zz_162_[16] = _zz_161_;
    _zz_162_[15] = _zz_161_;
    _zz_162_[14] = _zz_161_;
    _zz_162_[13] = _zz_161_;
    _zz_162_[12] = _zz_161_;
    _zz_162_[11] = _zz_161_;
    _zz_162_[10] = _zz_161_;
    _zz_162_[9] = _zz_161_;
    _zz_162_[8] = _zz_161_;
    _zz_162_[7] = _zz_161_;
    _zz_162_[6] = _zz_161_;
    _zz_162_[5] = _zz_161_;
    _zz_162_[4] = _zz_161_;
    _zz_162_[3] = _zz_161_;
    _zz_162_[2] = _zz_161_;
    _zz_162_[1] = _zz_161_;
    _zz_162_[0] = _zz_161_;
  end

  assign _zz_163_ = _zz_391_[11];
  always @ (*) begin
    _zz_164_[18] = _zz_163_;
    _zz_164_[17] = _zz_163_;
    _zz_164_[16] = _zz_163_;
    _zz_164_[15] = _zz_163_;
    _zz_164_[14] = _zz_163_;
    _zz_164_[13] = _zz_163_;
    _zz_164_[12] = _zz_163_;
    _zz_164_[11] = _zz_163_;
    _zz_164_[10] = _zz_163_;
    _zz_164_[9] = _zz_163_;
    _zz_164_[8] = _zz_163_;
    _zz_164_[7] = _zz_163_;
    _zz_164_[6] = _zz_163_;
    _zz_164_[5] = _zz_163_;
    _zz_164_[4] = _zz_163_;
    _zz_164_[3] = _zz_163_;
    _zz_164_[2] = _zz_163_;
    _zz_164_[1] = _zz_163_;
    _zz_164_[0] = _zz_163_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_165_ = {{_zz_160_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_165_ = {_zz_162_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_165_ = {{_zz_164_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BRANCH_SRC22);
  assign memory_BranchPlugin_predictionMissmatch = ((IBusSimplePlugin_fetchPrediction_cmd_hadBranch != memory_BRANCH_DO) || (memory_BRANCH_DO && memory_TARGET_MISSMATCH2));
  assign IBusSimplePlugin_fetchPrediction_rsp_wasRight = (! memory_BranchPlugin_predictionMissmatch);
  assign IBusSimplePlugin_fetchPrediction_rsp_finalPc = memory_BRANCH_CALC;
  assign IBusSimplePlugin_fetchPrediction_rsp_sourceLastWord = (((! memory_IS_RVC) && memory_PC[1]) ? memory_NEXT_PC2 : memory_PC);
  assign BranchPlugin_jumpInterface_valid = ((memory_arbitration_isValid && memory_BranchPlugin_predictionMissmatch) && (! 1'b0));
  assign BranchPlugin_jumpInterface_payload = (memory_BRANCH_DO ? memory_BRANCH_CALC : memory_NEXT_PC2);
  assign MmuPlugin_ports_0_cacheHits_0 = ((MmuPlugin_ports_0_cache_0_valid && (MmuPlugin_ports_0_cache_0_virtualAddress_1 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_0_cache_0_superPage || (MmuPlugin_ports_0_cache_0_virtualAddress_0 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_0_cacheHits_1 = ((MmuPlugin_ports_0_cache_1_valid && (MmuPlugin_ports_0_cache_1_virtualAddress_1 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_0_cache_1_superPage || (MmuPlugin_ports_0_cache_1_virtualAddress_0 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_0_cacheHits_2 = ((MmuPlugin_ports_0_cache_2_valid && (MmuPlugin_ports_0_cache_2_virtualAddress_1 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_0_cache_2_superPage || (MmuPlugin_ports_0_cache_2_virtualAddress_0 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_0_cacheHits_3 = ((MmuPlugin_ports_0_cache_3_valid && (MmuPlugin_ports_0_cache_3_virtualAddress_1 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_0_cache_3_superPage || (MmuPlugin_ports_0_cache_3_virtualAddress_0 == DBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_0_cacheHit = ({MmuPlugin_ports_0_cacheHits_3,{MmuPlugin_ports_0_cacheHits_2,{MmuPlugin_ports_0_cacheHits_1,MmuPlugin_ports_0_cacheHits_0}}} != (4'b0000));
  assign _zz_166_ = (MmuPlugin_ports_0_cacheHits_1 || MmuPlugin_ports_0_cacheHits_3);
  assign _zz_167_ = (MmuPlugin_ports_0_cacheHits_2 || MmuPlugin_ports_0_cacheHits_3);
  assign _zz_168_ = {_zz_167_,_zz_166_};
  assign MmuPlugin_ports_0_cacheLine_valid = _zz_200_;
  assign MmuPlugin_ports_0_cacheLine_exception = _zz_201_;
  assign MmuPlugin_ports_0_cacheLine_superPage = _zz_202_;
  assign MmuPlugin_ports_0_cacheLine_virtualAddress_0 = _zz_203_;
  assign MmuPlugin_ports_0_cacheLine_virtualAddress_1 = _zz_204_;
  assign MmuPlugin_ports_0_cacheLine_physicalAddress_0 = _zz_205_;
  assign MmuPlugin_ports_0_cacheLine_physicalAddress_1 = _zz_206_;
  assign MmuPlugin_ports_0_cacheLine_allowRead = _zz_207_;
  assign MmuPlugin_ports_0_cacheLine_allowWrite = _zz_208_;
  assign MmuPlugin_ports_0_cacheLine_allowExecute = _zz_209_;
  assign MmuPlugin_ports_0_cacheLine_allowUser = _zz_210_;
  always @ (*) begin
    MmuPlugin_ports_0_entryToReplace_willIncrement = 1'b0;
    if(_zz_255_)begin
      if(_zz_256_)begin
        MmuPlugin_ports_0_entryToReplace_willIncrement = 1'b1;
      end
    end
  end

  assign MmuPlugin_ports_0_entryToReplace_willClear = 1'b0;
  assign MmuPlugin_ports_0_entryToReplace_willOverflowIfInc = (MmuPlugin_ports_0_entryToReplace_value == (2'b11));
  assign MmuPlugin_ports_0_entryToReplace_willOverflow = (MmuPlugin_ports_0_entryToReplace_willOverflowIfInc && MmuPlugin_ports_0_entryToReplace_willIncrement);
  always @ (*) begin
    MmuPlugin_ports_0_entryToReplace_valueNext = (MmuPlugin_ports_0_entryToReplace_value + _zz_393_);
    if(MmuPlugin_ports_0_entryToReplace_willClear)begin
      MmuPlugin_ports_0_entryToReplace_valueNext = (2'b00);
    end
  end

  always @ (*) begin
    MmuPlugin_ports_0_requireMmuLockup = ((1'b1 && (! DBusSimplePlugin_mmuBus_cmd_bypassTranslation)) && MmuPlugin_satp_mode);
    if(((! MmuPlugin_status_mprv) && (CsrPlugin_privilege == (2'b11))))begin
      MmuPlugin_ports_0_requireMmuLockup = 1'b0;
    end
    if((CsrPlugin_privilege == (2'b11)))begin
      if(((! MmuPlugin_status_mprv) || (CsrPlugin_mstatus_MPP == (2'b11))))begin
        MmuPlugin_ports_0_requireMmuLockup = 1'b0;
      end
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_physicalAddress = {{MmuPlugin_ports_0_cacheLine_physicalAddress_1,(MmuPlugin_ports_0_cacheLine_superPage ? DBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12] : MmuPlugin_ports_0_cacheLine_physicalAddress_0)},DBusSimplePlugin_mmuBus_cmd_virtualAddress[11 : 0]};
    end else begin
      DBusSimplePlugin_mmuBus_rsp_physicalAddress = DBusSimplePlugin_mmuBus_cmd_virtualAddress;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_allowRead = (MmuPlugin_ports_0_cacheLine_allowRead || (MmuPlugin_status_mxr && MmuPlugin_ports_0_cacheLine_allowExecute));
    end else begin
      DBusSimplePlugin_mmuBus_rsp_allowRead = 1'b1;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_allowWrite = MmuPlugin_ports_0_cacheLine_allowWrite;
    end else begin
      DBusSimplePlugin_mmuBus_rsp_allowWrite = 1'b1;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_allowExecute = MmuPlugin_ports_0_cacheLine_allowExecute;
    end else begin
      DBusSimplePlugin_mmuBus_rsp_allowExecute = 1'b1;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_exception = (MmuPlugin_ports_0_cacheHit && ((MmuPlugin_ports_0_cacheLine_exception || ((MmuPlugin_ports_0_cacheLine_allowUser && (CsrPlugin_privilege == (2'b01))) && (! MmuPlugin_status_sum))) || ((! MmuPlugin_ports_0_cacheLine_allowUser) && (CsrPlugin_privilege == (2'b00)))));
    end else begin
      DBusSimplePlugin_mmuBus_rsp_exception = 1'b0;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_refilling = (! MmuPlugin_ports_0_cacheHit);
    end else begin
      DBusSimplePlugin_mmuBus_rsp_refilling = 1'b0;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_0_requireMmuLockup)begin
      DBusSimplePlugin_mmuBus_rsp_isPaging = 1'b1;
    end else begin
      DBusSimplePlugin_mmuBus_rsp_isPaging = 1'b0;
    end
  end

  assign DBusSimplePlugin_mmuBus_rsp_isIoAccess = (((DBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 28] == (4'b1011)) || (DBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 28] == (4'b1110))) || (DBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 28] == (4'b1111)));
  assign MmuPlugin_ports_1_cacheHits_0 = ((MmuPlugin_ports_1_cache_0_valid && (MmuPlugin_ports_1_cache_0_virtualAddress_1 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_1_cache_0_superPage || (MmuPlugin_ports_1_cache_0_virtualAddress_0 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_1_cacheHits_1 = ((MmuPlugin_ports_1_cache_1_valid && (MmuPlugin_ports_1_cache_1_virtualAddress_1 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_1_cache_1_superPage || (MmuPlugin_ports_1_cache_1_virtualAddress_0 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_1_cacheHits_2 = ((MmuPlugin_ports_1_cache_2_valid && (MmuPlugin_ports_1_cache_2_virtualAddress_1 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_1_cache_2_superPage || (MmuPlugin_ports_1_cache_2_virtualAddress_0 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_1_cacheHits_3 = ((MmuPlugin_ports_1_cache_3_valid && (MmuPlugin_ports_1_cache_3_virtualAddress_1 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22])) && (MmuPlugin_ports_1_cache_3_superPage || (MmuPlugin_ports_1_cache_3_virtualAddress_0 == IBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12])));
  assign MmuPlugin_ports_1_cacheHit = ({MmuPlugin_ports_1_cacheHits_3,{MmuPlugin_ports_1_cacheHits_2,{MmuPlugin_ports_1_cacheHits_1,MmuPlugin_ports_1_cacheHits_0}}} != (4'b0000));
  assign _zz_169_ = (MmuPlugin_ports_1_cacheHits_1 || MmuPlugin_ports_1_cacheHits_3);
  assign _zz_170_ = (MmuPlugin_ports_1_cacheHits_2 || MmuPlugin_ports_1_cacheHits_3);
  assign _zz_171_ = {_zz_170_,_zz_169_};
  assign MmuPlugin_ports_1_cacheLine_valid = _zz_211_;
  assign MmuPlugin_ports_1_cacheLine_exception = _zz_212_;
  assign MmuPlugin_ports_1_cacheLine_superPage = _zz_213_;
  assign MmuPlugin_ports_1_cacheLine_virtualAddress_0 = _zz_214_;
  assign MmuPlugin_ports_1_cacheLine_virtualAddress_1 = _zz_215_;
  assign MmuPlugin_ports_1_cacheLine_physicalAddress_0 = _zz_216_;
  assign MmuPlugin_ports_1_cacheLine_physicalAddress_1 = _zz_217_;
  assign MmuPlugin_ports_1_cacheLine_allowRead = _zz_218_;
  assign MmuPlugin_ports_1_cacheLine_allowWrite = _zz_219_;
  assign MmuPlugin_ports_1_cacheLine_allowExecute = _zz_220_;
  assign MmuPlugin_ports_1_cacheLine_allowUser = _zz_221_;
  always @ (*) begin
    MmuPlugin_ports_1_entryToReplace_willIncrement = 1'b0;
    if(_zz_255_)begin
      if(_zz_257_)begin
        MmuPlugin_ports_1_entryToReplace_willIncrement = 1'b1;
      end
    end
  end

  assign MmuPlugin_ports_1_entryToReplace_willClear = 1'b0;
  assign MmuPlugin_ports_1_entryToReplace_willOverflowIfInc = (MmuPlugin_ports_1_entryToReplace_value == (2'b11));
  assign MmuPlugin_ports_1_entryToReplace_willOverflow = (MmuPlugin_ports_1_entryToReplace_willOverflowIfInc && MmuPlugin_ports_1_entryToReplace_willIncrement);
  always @ (*) begin
    MmuPlugin_ports_1_entryToReplace_valueNext = (MmuPlugin_ports_1_entryToReplace_value + _zz_395_);
    if(MmuPlugin_ports_1_entryToReplace_willClear)begin
      MmuPlugin_ports_1_entryToReplace_valueNext = (2'b00);
    end
  end

  always @ (*) begin
    MmuPlugin_ports_1_requireMmuLockup = ((1'b1 && (! IBusSimplePlugin_mmuBus_cmd_bypassTranslation)) && MmuPlugin_satp_mode);
    if(((! MmuPlugin_status_mprv) && (CsrPlugin_privilege == (2'b11))))begin
      MmuPlugin_ports_1_requireMmuLockup = 1'b0;
    end
    if((CsrPlugin_privilege == (2'b11)))begin
      MmuPlugin_ports_1_requireMmuLockup = 1'b0;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_physicalAddress = {{MmuPlugin_ports_1_cacheLine_physicalAddress_1,(MmuPlugin_ports_1_cacheLine_superPage ? IBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12] : MmuPlugin_ports_1_cacheLine_physicalAddress_0)},IBusSimplePlugin_mmuBus_cmd_virtualAddress[11 : 0]};
    end else begin
      IBusSimplePlugin_mmuBus_rsp_physicalAddress = IBusSimplePlugin_mmuBus_cmd_virtualAddress;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_allowRead = (MmuPlugin_ports_1_cacheLine_allowRead || (MmuPlugin_status_mxr && MmuPlugin_ports_1_cacheLine_allowExecute));
    end else begin
      IBusSimplePlugin_mmuBus_rsp_allowRead = 1'b1;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_allowWrite = MmuPlugin_ports_1_cacheLine_allowWrite;
    end else begin
      IBusSimplePlugin_mmuBus_rsp_allowWrite = 1'b1;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_allowExecute = MmuPlugin_ports_1_cacheLine_allowExecute;
    end else begin
      IBusSimplePlugin_mmuBus_rsp_allowExecute = 1'b1;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_exception = (MmuPlugin_ports_1_cacheHit && ((MmuPlugin_ports_1_cacheLine_exception || ((MmuPlugin_ports_1_cacheLine_allowUser && (CsrPlugin_privilege == (2'b01))) && (! MmuPlugin_status_sum))) || ((! MmuPlugin_ports_1_cacheLine_allowUser) && (CsrPlugin_privilege == (2'b00)))));
    end else begin
      IBusSimplePlugin_mmuBus_rsp_exception = 1'b0;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_refilling = (! MmuPlugin_ports_1_cacheHit);
    end else begin
      IBusSimplePlugin_mmuBus_rsp_refilling = 1'b0;
    end
  end

  always @ (*) begin
    if(MmuPlugin_ports_1_requireMmuLockup)begin
      IBusSimplePlugin_mmuBus_rsp_isPaging = 1'b1;
    end else begin
      IBusSimplePlugin_mmuBus_rsp_isPaging = 1'b0;
    end
  end

  assign IBusSimplePlugin_mmuBus_rsp_isIoAccess = (((IBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 28] == (4'b1011)) || (IBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 28] == (4'b1110))) || (IBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 28] == (4'b1111)));
  assign MmuPlugin_shared_dBusRsp_pte_V = _zz_396_[0];
  assign MmuPlugin_shared_dBusRsp_pte_R = _zz_397_[0];
  assign MmuPlugin_shared_dBusRsp_pte_W = _zz_398_[0];
  assign MmuPlugin_shared_dBusRsp_pte_X = _zz_399_[0];
  assign MmuPlugin_shared_dBusRsp_pte_U = _zz_400_[0];
  assign MmuPlugin_shared_dBusRsp_pte_G = _zz_401_[0];
  assign MmuPlugin_shared_dBusRsp_pte_A = _zz_402_[0];
  assign MmuPlugin_shared_dBusRsp_pte_D = _zz_403_[0];
  assign MmuPlugin_shared_dBusRsp_pte_RSW = MmuPlugin_dBusAccess_rsp_payload_data[9 : 8];
  assign MmuPlugin_shared_dBusRsp_pte_PPN0 = MmuPlugin_dBusAccess_rsp_payload_data[19 : 10];
  assign MmuPlugin_shared_dBusRsp_pte_PPN1 = MmuPlugin_dBusAccess_rsp_payload_data[31 : 20];
  assign MmuPlugin_shared_dBusRsp_exception = (((! MmuPlugin_shared_dBusRsp_pte_V) || ((! MmuPlugin_shared_dBusRsp_pte_R) && MmuPlugin_shared_dBusRsp_pte_W)) || MmuPlugin_dBusAccess_rsp_payload_error);
  assign MmuPlugin_shared_dBusRsp_leaf = (MmuPlugin_shared_dBusRsp_pte_R || MmuPlugin_shared_dBusRsp_pte_X);
  always @ (*) begin
    MmuPlugin_dBusAccess_cmd_valid = 1'b0;
    case(MmuPlugin_shared_state_1_)
      `MmuPlugin_shared_State_defaultEncoding_IDLE : begin
      end
      `MmuPlugin_shared_State_defaultEncoding_L1_CMD : begin
        MmuPlugin_dBusAccess_cmd_valid = 1'b1;
      end
      `MmuPlugin_shared_State_defaultEncoding_L1_RSP : begin
      end
      `MmuPlugin_shared_State_defaultEncoding_L0_CMD : begin
        MmuPlugin_dBusAccess_cmd_valid = 1'b1;
      end
      default : begin
      end
    endcase
  end

  assign MmuPlugin_dBusAccess_cmd_payload_write = 1'b0;
  assign MmuPlugin_dBusAccess_cmd_payload_size = (2'b10);
  always @ (*) begin
    MmuPlugin_dBusAccess_cmd_payload_address = 32'h0;
    case(MmuPlugin_shared_state_1_)
      `MmuPlugin_shared_State_defaultEncoding_IDLE : begin
      end
      `MmuPlugin_shared_State_defaultEncoding_L1_CMD : begin
        MmuPlugin_dBusAccess_cmd_payload_address = {{MmuPlugin_satp_ppn,MmuPlugin_shared_vpn_1},(2'b00)};
      end
      `MmuPlugin_shared_State_defaultEncoding_L1_RSP : begin
      end
      `MmuPlugin_shared_State_defaultEncoding_L0_CMD : begin
        MmuPlugin_dBusAccess_cmd_payload_address = {{{MmuPlugin_shared_pteBuffer_PPN1[9 : 0],MmuPlugin_shared_pteBuffer_PPN0},MmuPlugin_shared_vpn_0},(2'b00)};
      end
      default : begin
      end
    endcase
  end

  assign MmuPlugin_dBusAccess_cmd_payload_data = 32'h0;
  assign MmuPlugin_dBusAccess_cmd_payload_writeMask = (4'bxxxx);
  assign DBusSimplePlugin_mmuBus_busy = ((MmuPlugin_shared_state_1_ != `MmuPlugin_shared_State_defaultEncoding_IDLE) && (MmuPlugin_shared_portId == (1'b1)));
  assign IBusSimplePlugin_mmuBus_busy = ((MmuPlugin_shared_state_1_ != `MmuPlugin_shared_State_defaultEncoding_IDLE) && (MmuPlugin_shared_portId == (1'b0)));
  assign _zz_34_ = _zz_44_;
  assign _zz_32_ = _zz_45_;
  assign _zz_21_ = decode_ENV_CTRL;
  assign _zz_18_ = execute_ENV_CTRL;
  assign _zz_16_ = memory_ENV_CTRL;
  assign _zz_19_ = _zz_46_;
  assign _zz_24_ = decode_to_execute_ENV_CTRL;
  assign _zz_23_ = execute_to_memory_ENV_CTRL;
  assign _zz_25_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_14_ = decode_ALU_BITWISE_CTRL;
  assign _zz_12_ = _zz_40_;
  assign _zz_36_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_11_ = decode_ALU_CTRL;
  assign _zz_9_ = _zz_42_;
  assign _zz_35_ = decode_to_execute_ALU_CTRL;
  assign _zz_8_ = decode_SHIFT_CTRL;
  assign _zz_5_ = execute_SHIFT_CTRL;
  assign _zz_6_ = _zz_41_;
  assign _zz_29_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_28_ = execute_to_memory_SHIFT_CTRL;
  assign _zz_3_ = decode_BRANCH_CTRL;
  assign _zz_1_ = _zz_43_;
  assign _zz_22_ = decode_to_execute_BRANCH_CTRL;
  assign decode_arbitration_isFlushed = (({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,execute_arbitration_flushNext}} != (3'b000)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,{execute_arbitration_flushIt,decode_arbitration_flushIt}}} != (4'b0000)));
  assign execute_arbitration_isFlushed = (({writeBack_arbitration_flushNext,memory_arbitration_flushNext} != (2'b00)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,execute_arbitration_flushIt}} != (3'b000)));
  assign memory_arbitration_isFlushed = ((writeBack_arbitration_flushNext != (1'b0)) || ({writeBack_arbitration_flushIt,memory_arbitration_flushIt} != (2'b00)));
  assign writeBack_arbitration_isFlushed = (1'b0 || (writeBack_arbitration_flushIt != (1'b0)));
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  always @ (*) begin
    IBusSimplePlugin_injectionPort_ready = 1'b0;
    case(_zz_172_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
      end
      3'b011 : begin
      end
      3'b100 : begin
        IBusSimplePlugin_injectionPort_ready = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_173_ = 32'h0;
    if(execute_CsrPlugin_csr_3857)begin
      _zz_173_[0 : 0] = (1'b1);
    end
  end

  always @ (*) begin
    _zz_174_ = 32'h0;
    if(execute_CsrPlugin_csr_3858)begin
      _zz_174_[1 : 0] = (2'b10);
    end
  end

  always @ (*) begin
    _zz_175_ = 32'h0;
    if(execute_CsrPlugin_csr_3859)begin
      _zz_175_[1 : 0] = (2'b11);
    end
  end

  always @ (*) begin
    _zz_176_ = 32'h0;
    if(execute_CsrPlugin_csr_768)begin
      _zz_176_[12 : 11] = CsrPlugin_mstatus_MPP;
      _zz_176_[7 : 7] = CsrPlugin_mstatus_MPIE;
      _zz_176_[3 : 3] = CsrPlugin_mstatus_MIE;
      _zz_176_[8 : 8] = CsrPlugin_sstatus_SPP;
      _zz_176_[5 : 5] = CsrPlugin_sstatus_SPIE;
      _zz_176_[1 : 1] = CsrPlugin_sstatus_SIE;
      _zz_176_[19 : 19] = MmuPlugin_status_mxr;
      _zz_176_[18 : 18] = MmuPlugin_status_sum;
      _zz_176_[17 : 17] = MmuPlugin_status_mprv;
    end
  end

  always @ (*) begin
    _zz_177_ = 32'h0;
    if(execute_CsrPlugin_csr_836)begin
      _zz_177_[11 : 11] = CsrPlugin_mip_MEIP;
      _zz_177_[7 : 7] = CsrPlugin_mip_MTIP;
      _zz_177_[3 : 3] = CsrPlugin_mip_MSIP;
      _zz_177_[5 : 5] = CsrPlugin_sip_STIP;
      _zz_177_[1 : 1] = CsrPlugin_sip_SSIP;
      _zz_177_[9 : 9] = CsrPlugin_sip_SEIP_OR;
    end
  end

  always @ (*) begin
    _zz_178_ = 32'h0;
    if(execute_CsrPlugin_csr_772)begin
      _zz_178_[11 : 11] = CsrPlugin_mie_MEIE;
      _zz_178_[7 : 7] = CsrPlugin_mie_MTIE;
      _zz_178_[3 : 3] = CsrPlugin_mie_MSIE;
      _zz_178_[9 : 9] = CsrPlugin_sie_SEIE;
      _zz_178_[5 : 5] = CsrPlugin_sie_STIE;
      _zz_178_[1 : 1] = CsrPlugin_sie_SSIE;
    end
  end

  always @ (*) begin
    _zz_179_ = 32'h0;
    if(execute_CsrPlugin_csr_833)begin
      _zz_179_[31 : 0] = CsrPlugin_mepc;
    end
  end

  always @ (*) begin
    _zz_180_ = 32'h0;
    if(execute_CsrPlugin_csr_832)begin
      _zz_180_[31 : 0] = CsrPlugin_mscratch;
    end
  end

  always @ (*) begin
    _zz_181_ = 32'h0;
    if(execute_CsrPlugin_csr_834)begin
      _zz_181_[31 : 31] = CsrPlugin_mcause_interrupt;
      _zz_181_[3 : 0] = CsrPlugin_mcause_exceptionCode;
    end
  end

  always @ (*) begin
    _zz_182_ = 32'h0;
    if(execute_CsrPlugin_csr_835)begin
      _zz_182_[31 : 0] = CsrPlugin_mtval;
    end
  end

  always @ (*) begin
    _zz_183_ = 32'h0;
    if(execute_CsrPlugin_csr_256)begin
      _zz_183_[8 : 8] = CsrPlugin_sstatus_SPP;
      _zz_183_[5 : 5] = CsrPlugin_sstatus_SPIE;
      _zz_183_[1 : 1] = CsrPlugin_sstatus_SIE;
      _zz_183_[19 : 19] = MmuPlugin_status_mxr;
      _zz_183_[18 : 18] = MmuPlugin_status_sum;
      _zz_183_[17 : 17] = MmuPlugin_status_mprv;
    end
  end

  always @ (*) begin
    _zz_184_ = 32'h0;
    if(execute_CsrPlugin_csr_324)begin
      _zz_184_[5 : 5] = CsrPlugin_sip_STIP;
      _zz_184_[1 : 1] = CsrPlugin_sip_SSIP;
      _zz_184_[9 : 9] = CsrPlugin_sip_SEIP_OR;
    end
  end

  always @ (*) begin
    _zz_185_ = 32'h0;
    if(execute_CsrPlugin_csr_260)begin
      _zz_185_[9 : 9] = CsrPlugin_sie_SEIE;
      _zz_185_[5 : 5] = CsrPlugin_sie_STIE;
      _zz_185_[1 : 1] = CsrPlugin_sie_SSIE;
    end
  end

  always @ (*) begin
    _zz_186_ = 32'h0;
    if(execute_CsrPlugin_csr_261)begin
      _zz_186_[31 : 2] = CsrPlugin_stvec_base;
      _zz_186_[1 : 0] = CsrPlugin_stvec_mode;
    end
  end

  always @ (*) begin
    _zz_187_ = 32'h0;
    if(execute_CsrPlugin_csr_321)begin
      _zz_187_[31 : 0] = CsrPlugin_sepc;
    end
  end

  always @ (*) begin
    _zz_188_ = 32'h0;
    if(execute_CsrPlugin_csr_320)begin
      _zz_188_[31 : 0] = CsrPlugin_sscratch;
    end
  end

  always @ (*) begin
    _zz_189_ = 32'h0;
    if(execute_CsrPlugin_csr_322)begin
      _zz_189_[31 : 31] = CsrPlugin_scause_interrupt;
      _zz_189_[3 : 0] = CsrPlugin_scause_exceptionCode;
    end
  end

  always @ (*) begin
    _zz_190_ = 32'h0;
    if(execute_CsrPlugin_csr_323)begin
      _zz_190_[31 : 0] = CsrPlugin_stval;
    end
  end

  always @ (*) begin
    _zz_191_ = 32'h0;
    if(execute_CsrPlugin_csr_384)begin
      _zz_191_[31 : 31] = MmuPlugin_satp_mode;
      _zz_191_[19 : 0] = MmuPlugin_satp_ppn;
    end
  end

  assign execute_CsrPlugin_readData = (((((_zz_173_ | _zz_174_) | (_zz_175_ | _zz_662_)) | ((_zz_176_ | _zz_177_) | (_zz_178_ | _zz_179_))) | (((_zz_180_ | _zz_181_) | (_zz_182_ | _zz_183_)) | ((_zz_184_ | _zz_185_) | (_zz_186_ | _zz_187_)))) | ((_zz_188_ | _zz_189_) | (_zz_190_ | _zz_191_)));
  assign iBus_cmd_ready = ((1'b1 && (! iBus_cmd_m2sPipe_valid)) || iBus_cmd_m2sPipe_ready);
  assign iBus_cmd_m2sPipe_valid = iBus_cmd_m2sPipe_rValid;
  assign iBus_cmd_m2sPipe_payload_pc = iBus_cmd_m2sPipe_rData_pc;
  assign iBusWishbone_ADR = (iBus_cmd_m2sPipe_payload_pc >>> 2);
  assign iBusWishbone_CTI = (3'b000);
  assign iBusWishbone_BTE = (2'b00);
  assign iBusWishbone_SEL = (4'b1111);
  assign iBusWishbone_WE = 1'b0;
  assign iBusWishbone_DAT_MOSI = 32'h0;
  assign iBusWishbone_CYC = iBus_cmd_m2sPipe_valid;
  assign iBusWishbone_STB = iBus_cmd_m2sPipe_valid;
  assign iBus_cmd_m2sPipe_ready = (iBus_cmd_m2sPipe_valid && iBusWishbone_ACK);
  assign iBus_rsp_valid = (iBusWishbone_CYC && iBusWishbone_ACK);
  assign iBus_rsp_payload_inst = iBusWishbone_DAT_MISO;
  assign iBus_rsp_payload_error = 1'b0;
  assign dBus_cmd_halfPipe_valid = dBus_cmd_halfPipe_regs_valid;
  assign dBus_cmd_halfPipe_payload_wr = dBus_cmd_halfPipe_regs_payload_wr;
  assign dBus_cmd_halfPipe_payload_address = dBus_cmd_halfPipe_regs_payload_address;
  assign dBus_cmd_halfPipe_payload_data = dBus_cmd_halfPipe_regs_payload_data;
  assign dBus_cmd_halfPipe_payload_size = dBus_cmd_halfPipe_regs_payload_size;
  assign dBus_cmd_ready = dBus_cmd_halfPipe_regs_ready;
  assign dBusWishbone_ADR = (dBus_cmd_halfPipe_payload_address >>> 2);
  assign dBusWishbone_CTI = (3'b000);
  assign dBusWishbone_BTE = (2'b00);
  always @ (*) begin
    case(dBus_cmd_halfPipe_payload_size)
      2'b00 : begin
        _zz_192_ = (4'b0001);
      end
      2'b01 : begin
        _zz_192_ = (4'b0011);
      end
      default : begin
        _zz_192_ = (4'b1111);
      end
    endcase
  end

  always @ (*) begin
    dBusWishbone_SEL = (_zz_192_ <<< dBus_cmd_halfPipe_payload_address[1 : 0]);
    if((! dBus_cmd_halfPipe_payload_wr))begin
      dBusWishbone_SEL = (4'b1111);
    end
  end

  assign dBusWishbone_WE = dBus_cmd_halfPipe_payload_wr;
  assign dBusWishbone_DAT_MOSI = dBus_cmd_halfPipe_payload_data;
  assign dBus_cmd_halfPipe_ready = (dBus_cmd_halfPipe_valid && dBusWishbone_ACK);
  assign dBusWishbone_CYC = dBus_cmd_halfPipe_valid;
  assign dBusWishbone_STB = dBus_cmd_halfPipe_valid;
  assign dBus_rsp_ready = ((dBus_cmd_halfPipe_valid && (! dBusWishbone_WE)) && dBusWishbone_ACK);
  assign dBus_rsp_data = dBusWishbone_DAT_MISO;
  assign dBus_rsp_error = 1'b0;
  assign debug_bus_cmd_valid = systemDebugger_1__io_mem_cmd_valid;
  assign debug_bus_cmd_payload_wr = systemDebugger_1__io_mem_cmd_payload_wr;
  assign debug_bus_cmd_payload_address = systemDebugger_1__io_mem_cmd_payload_address[7:0];
  assign debug_bus_cmd_payload_data = systemDebugger_1__io_mem_cmd_payload_data;
  assign jtag_tdo = jtagBridge_1__io_jtag_tdo;
  assign _zz_195_ = 1'b0;
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      IBusSimplePlugin_fetchPc_pcReg <= 32'h80000000;
      IBusSimplePlugin_fetchPc_correctionReg <= 1'b0;
      IBusSimplePlugin_fetchPc_booted <= 1'b0;
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      IBusSimplePlugin_decodePc_pcReg <= 32'h80000000;
      _zz_60_ <= 1'b0;
      IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
      IBusSimplePlugin_decompressor_throw2BytesReg <= 1'b0;
      _zz_89_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      IBusSimplePlugin_pending_value <= (3'b000);
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (3'b000);
      execute_DBusSimplePlugin_atomic_reserved <= 1'b0;
      _zz_104_ <= (2'b00);
      _zz_119_ <= 1'b1;
      _zz_131_ <= 1'b0;
      memory_MulDivIterativePlugin_div_counter_value <= 6'h0;
      _zz_145_ <= (2'b11);
      CsrPlugin_mtvec_mode <= (2'b00);
      CsrPlugin_mtvec_base <= 30'h20000008;
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_medeleg_IAM <= 1'b0;
      CsrPlugin_medeleg_IAF <= 1'b0;
      CsrPlugin_medeleg_II <= 1'b0;
      CsrPlugin_medeleg_LAM <= 1'b0;
      CsrPlugin_medeleg_LAF <= 1'b0;
      CsrPlugin_medeleg_SAM <= 1'b0;
      CsrPlugin_medeleg_SAF <= 1'b0;
      CsrPlugin_medeleg_EU <= 1'b0;
      CsrPlugin_medeleg_ES <= 1'b0;
      CsrPlugin_medeleg_IPF <= 1'b0;
      CsrPlugin_medeleg_LPF <= 1'b0;
      CsrPlugin_medeleg_SPF <= 1'b0;
      CsrPlugin_mideleg_ST <= 1'b0;
      CsrPlugin_mideleg_SE <= 1'b0;
      CsrPlugin_mideleg_SS <= 1'b0;
      CsrPlugin_sstatus_SIE <= 1'b0;
      CsrPlugin_sstatus_SPIE <= 1'b0;
      CsrPlugin_sstatus_SPP <= (1'b1);
      CsrPlugin_sip_SEIP_SOFT <= 1'b0;
      CsrPlugin_sip_STIP <= 1'b0;
      CsrPlugin_sip_SSIP <= 1'b0;
      CsrPlugin_sie_SEIE <= 1'b0;
      CsrPlugin_sie_STIE <= 1'b0;
      CsrPlugin_sie_SSIE <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      CsrPlugin_interrupt_valid <= 1'b0;
      CsrPlugin_lastStageWasWfi <= 1'b0;
      CsrPlugin_pipelineLiberator_pcValids_0 <= 1'b0;
      CsrPlugin_pipelineLiberator_pcValids_1 <= 1'b0;
      CsrPlugin_pipelineLiberator_pcValids_2 <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      execute_CsrPlugin_wfiWake <= 1'b0;
      MmuPlugin_status_sum <= 1'b0;
      MmuPlugin_status_mxr <= 1'b0;
      MmuPlugin_status_mprv <= 1'b0;
      MmuPlugin_satp_mode <= 1'b0;
      MmuPlugin_ports_0_cache_0_valid <= 1'b0;
      MmuPlugin_ports_0_cache_1_valid <= 1'b0;
      MmuPlugin_ports_0_cache_2_valid <= 1'b0;
      MmuPlugin_ports_0_cache_3_valid <= 1'b0;
      MmuPlugin_ports_0_entryToReplace_value <= (2'b00);
      MmuPlugin_ports_1_cache_0_valid <= 1'b0;
      MmuPlugin_ports_1_cache_1_valid <= 1'b0;
      MmuPlugin_ports_1_cache_2_valid <= 1'b0;
      MmuPlugin_ports_1_cache_3_valid <= 1'b0;
      MmuPlugin_ports_1_entryToReplace_value <= (2'b00);
      MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_IDLE;
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      _zz_172_ <= (3'b000);
      memory_to_writeBack_REGFILE_WRITE_DATA <= 32'h0;
      memory_to_writeBack_INSTRUCTION <= 32'h0;
      iBus_cmd_m2sPipe_rValid <= 1'b0;
      dBus_cmd_halfPipe_regs_valid <= 1'b0;
      dBus_cmd_halfPipe_regs_ready <= 1'b1;
    end else begin
      if(IBusSimplePlugin_fetchPc_correction)begin
        IBusSimplePlugin_fetchPc_correctionReg <= 1'b1;
      end
      if((IBusSimplePlugin_fetchPc_output_valid && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_correctionReg <= 1'b0;
      end
      IBusSimplePlugin_fetchPc_booted <= 1'b1;
      if((IBusSimplePlugin_fetchPc_correction || IBusSimplePlugin_fetchPc_pcRegPropagate))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusSimplePlugin_fetchPc_output_valid && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(((! IBusSimplePlugin_fetchPc_output_valid) && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusSimplePlugin_fetchPc_booted && ((IBusSimplePlugin_fetchPc_output_ready || IBusSimplePlugin_fetchPc_correction) || IBusSimplePlugin_fetchPc_pcRegPropagate)))begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      if((decode_arbitration_isFiring && (! IBusSimplePlugin_decodePc_injectedDecode)))begin
        IBusSimplePlugin_decodePc_pcReg <= IBusSimplePlugin_decodePc_pcPlus;
      end
      if(IBusSimplePlugin_decodePc_predictionPcLoad_valid)begin
        IBusSimplePlugin_decodePc_pcReg <= IBusSimplePlugin_decodePc_predictionPcLoad_payload;
      end
      if(_zz_236_)begin
        IBusSimplePlugin_decodePc_pcReg <= IBusSimplePlugin_jump_pcLoad_payload;
      end
      if(IBusSimplePlugin_iBusRsp_flush)begin
        _zz_60_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
        _zz_60_ <= (IBusSimplePlugin_iBusRsp_stages_0_output_valid && (! 1'b0));
      end
      if((IBusSimplePlugin_decompressor_output_valid && IBusSimplePlugin_decompressor_output_ready))begin
        IBusSimplePlugin_decompressor_throw2BytesReg <= ((((! IBusSimplePlugin_decompressor_unaligned) && IBusSimplePlugin_decompressor_isInputLowRvc) && IBusSimplePlugin_decompressor_isInputHighRvc) || (IBusSimplePlugin_decompressor_bufferValid && IBusSimplePlugin_decompressor_isInputHighRvc));
      end
      if((IBusSimplePlugin_decompressor_output_ready && IBusSimplePlugin_decompressor_input_valid))begin
        IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
      end
      if(_zz_258_)begin
        if(IBusSimplePlugin_decompressor_bufferFill)begin
          IBusSimplePlugin_decompressor_bufferValid <= 1'b1;
        end
      end
      if((IBusSimplePlugin_externalFlush || IBusSimplePlugin_decompressor_consumeCurrent))begin
        IBusSimplePlugin_decompressor_throw2BytesReg <= 1'b0;
        IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        _zz_89_ <= 1'b0;
      end
      if(IBusSimplePlugin_decompressor_output_ready)begin
        _zz_89_ <= (IBusSimplePlugin_decompressor_output_valid && (! IBusSimplePlugin_externalFlush));
      end
      if((! 1'b0))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if(IBusSimplePlugin_decodePc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end
      if(IBusSimplePlugin_decodePc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end
      if(IBusSimplePlugin_decodePc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
      end
      if(IBusSimplePlugin_decodePc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if(_zz_237_)begin
        IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
        IBusSimplePlugin_decompressor_throw2BytesReg <= 1'b0;
      end
      IBusSimplePlugin_pending_value <= IBusSimplePlugin_pending_next;
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter - _zz_351_);
      if(IBusSimplePlugin_iBusRsp_flush)begin
        IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_pending_value - _zz_353_);
      end
      if((((execute_arbitration_isFiring && execute_MEMORY_ENABLE) && execute_MEMORY_ATOMIC) && (! execute_MEMORY_STORE)))begin
        execute_DBusSimplePlugin_atomic_reserved <= 1'b1;
      end
      if(contextSwitching)begin
        execute_DBusSimplePlugin_atomic_reserved <= 1'b0;
      end
      case(_zz_104_)
        2'b00 : begin
          if(MmuPlugin_dBusAccess_cmd_valid)begin
            if((! ({writeBack_arbitration_isValid,{memory_arbitration_isValid,execute_arbitration_isValid}} != (3'b000))))begin
              _zz_104_ <= (2'b01);
            end
          end
        end
        2'b01 : begin
          if(dBus_cmd_ready)begin
            _zz_104_ <= (MmuPlugin_dBusAccess_cmd_payload_write ? (2'b00) : (2'b10));
          end
        end
        2'b10 : begin
          if(dBus_rsp_ready)begin
            _zz_104_ <= (2'b00);
          end
        end
        default : begin
        end
      endcase
      _zz_119_ <= 1'b0;
      _zz_131_ <= (_zz_38_ && writeBack_arbitration_isFiring);
      memory_MulDivIterativePlugin_div_counter_value <= memory_MulDivIterativePlugin_div_counter_valueNext;
      if((! decode_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= 1'b0;
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
      end
      if((! execute_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= (CsrPlugin_exceptionPortCtrl_exceptionValids_decode && (! decode_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
      end
      if((! memory_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= (CsrPlugin_exceptionPortCtrl_exceptionValids_execute && (! execute_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
      end
      if((! writeBack_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= (CsrPlugin_exceptionPortCtrl_exceptionValids_memory && (! memory_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      end
      CsrPlugin_interrupt_valid <= 1'b0;
      if(_zz_259_)begin
        if(_zz_260_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_261_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_262_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
      end
      if(_zz_263_)begin
        if(_zz_264_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_265_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_266_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_267_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_268_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_269_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
      end
      CsrPlugin_lastStageWasWfi <= (writeBack_arbitration_isFiring && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_WFI));
      if(CsrPlugin_pipelineLiberator_active)begin
        if((! execute_arbitration_isStuck))begin
          CsrPlugin_pipelineLiberator_pcValids_0 <= 1'b1;
        end
        if((! memory_arbitration_isStuck))begin
          CsrPlugin_pipelineLiberator_pcValids_1 <= CsrPlugin_pipelineLiberator_pcValids_0;
        end
        if((! writeBack_arbitration_isStuck))begin
          CsrPlugin_pipelineLiberator_pcValids_2 <= CsrPlugin_pipelineLiberator_pcValids_1;
        end
      end
      if(((! CsrPlugin_pipelineLiberator_active) || decode_arbitration_removeIt))begin
        CsrPlugin_pipelineLiberator_pcValids_0 <= 1'b0;
        CsrPlugin_pipelineLiberator_pcValids_1 <= 1'b0;
        CsrPlugin_pipelineLiberator_pcValids_2 <= 1'b0;
      end
      if(CsrPlugin_interruptJump)begin
        CsrPlugin_interrupt_valid <= 1'b0;
      end
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_232_)begin
        _zz_145_ <= CsrPlugin_targetPrivilege;
        case(CsrPlugin_targetPrivilege)
          2'b01 : begin
            CsrPlugin_sstatus_SIE <= 1'b0;
            CsrPlugin_sstatus_SPIE <= CsrPlugin_sstatus_SIE;
            CsrPlugin_sstatus_SPP <= CsrPlugin_privilege[0 : 0];
          end
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= 1'b0;
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          end
          default : begin
          end
        endcase
      end
      if(_zz_233_)begin
        case(_zz_235_)
          2'b11 : begin
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= 1'b1;
            _zz_145_ <= CsrPlugin_mstatus_MPP;
          end
          2'b01 : begin
            CsrPlugin_sstatus_SPP <= (1'b0);
            CsrPlugin_sstatus_SIE <= CsrPlugin_sstatus_SPIE;
            CsrPlugin_sstatus_SPIE <= 1'b1;
            _zz_145_ <= {(1'b0),CsrPlugin_sstatus_SPP};
          end
          default : begin
          end
        endcase
      end
      execute_CsrPlugin_wfiWake <= (({_zz_151_,{_zz_150_,{_zz_149_,{_zz_148_,{_zz_147_,_zz_146_}}}}} != 6'h0) || CsrPlugin_thirdPartyWake);
      MmuPlugin_ports_0_entryToReplace_value <= MmuPlugin_ports_0_entryToReplace_valueNext;
      if(contextSwitching)begin
        if(MmuPlugin_ports_0_cache_0_exception)begin
          MmuPlugin_ports_0_cache_0_valid <= 1'b0;
        end
        if(MmuPlugin_ports_0_cache_1_exception)begin
          MmuPlugin_ports_0_cache_1_valid <= 1'b0;
        end
        if(MmuPlugin_ports_0_cache_2_exception)begin
          MmuPlugin_ports_0_cache_2_valid <= 1'b0;
        end
        if(MmuPlugin_ports_0_cache_3_exception)begin
          MmuPlugin_ports_0_cache_3_valid <= 1'b0;
        end
      end
      MmuPlugin_ports_1_entryToReplace_value <= MmuPlugin_ports_1_entryToReplace_valueNext;
      if(contextSwitching)begin
        if(MmuPlugin_ports_1_cache_0_exception)begin
          MmuPlugin_ports_1_cache_0_valid <= 1'b0;
        end
        if(MmuPlugin_ports_1_cache_1_exception)begin
          MmuPlugin_ports_1_cache_1_valid <= 1'b0;
        end
        if(MmuPlugin_ports_1_cache_2_exception)begin
          MmuPlugin_ports_1_cache_2_valid <= 1'b0;
        end
        if(MmuPlugin_ports_1_cache_3_exception)begin
          MmuPlugin_ports_1_cache_3_valid <= 1'b0;
        end
      end
      case(MmuPlugin_shared_state_1_)
        `MmuPlugin_shared_State_defaultEncoding_IDLE : begin
          if(_zz_270_)begin
            MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L1_CMD;
          end
          if(_zz_271_)begin
            MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L1_CMD;
          end
        end
        `MmuPlugin_shared_State_defaultEncoding_L1_CMD : begin
          if(MmuPlugin_dBusAccess_cmd_ready)begin
            MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L1_RSP;
          end
        end
        `MmuPlugin_shared_State_defaultEncoding_L1_RSP : begin
          if(MmuPlugin_dBusAccess_rsp_valid)begin
            MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L0_CMD;
            if((MmuPlugin_shared_dBusRsp_leaf || MmuPlugin_shared_dBusRsp_exception))begin
              MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_IDLE;
            end
            if(MmuPlugin_dBusAccess_rsp_payload_redo)begin
              MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L1_CMD;
            end
          end
        end
        `MmuPlugin_shared_State_defaultEncoding_L0_CMD : begin
          if(MmuPlugin_dBusAccess_cmd_ready)begin
            MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L0_RSP;
          end
        end
        default : begin
          if(MmuPlugin_dBusAccess_rsp_valid)begin
            MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_IDLE;
            if(MmuPlugin_dBusAccess_rsp_payload_redo)begin
              MmuPlugin_shared_state_1_ <= `MmuPlugin_shared_State_defaultEncoding_L0_CMD;
            end
          end
        end
      endcase
      if(_zz_255_)begin
        if(_zz_256_)begin
          if(_zz_272_)begin
            MmuPlugin_ports_0_cache_0_valid <= 1'b1;
          end
          if(_zz_273_)begin
            MmuPlugin_ports_0_cache_1_valid <= 1'b1;
          end
          if(_zz_274_)begin
            MmuPlugin_ports_0_cache_2_valid <= 1'b1;
          end
          if(_zz_275_)begin
            MmuPlugin_ports_0_cache_3_valid <= 1'b1;
          end
        end
        if(_zz_257_)begin
          if(_zz_276_)begin
            MmuPlugin_ports_1_cache_0_valid <= 1'b1;
          end
          if(_zz_277_)begin
            MmuPlugin_ports_1_cache_1_valid <= 1'b1;
          end
          if(_zz_278_)begin
            MmuPlugin_ports_1_cache_2_valid <= 1'b1;
          end
          if(_zz_279_)begin
            MmuPlugin_ports_1_cache_3_valid <= 1'b1;
          end
        end
      end
      if((writeBack_arbitration_isValid && writeBack_IS_SFENCE_VMA))begin
        MmuPlugin_ports_0_cache_0_valid <= 1'b0;
        MmuPlugin_ports_0_cache_1_valid <= 1'b0;
        MmuPlugin_ports_0_cache_2_valid <= 1'b0;
        MmuPlugin_ports_0_cache_3_valid <= 1'b0;
        MmuPlugin_ports_1_cache_0_valid <= 1'b0;
        MmuPlugin_ports_1_cache_1_valid <= 1'b0;
        MmuPlugin_ports_1_cache_2_valid <= 1'b0;
        MmuPlugin_ports_1_cache_3_valid <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= _zz_27_;
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(_zz_172_)
        3'b000 : begin
          if(IBusSimplePlugin_injectionPort_valid)begin
            _zz_172_ <= (3'b001);
          end
        end
        3'b001 : begin
          _zz_172_ <= (3'b010);
        end
        3'b010 : begin
          _zz_172_ <= (3'b011);
        end
        3'b011 : begin
          if((! decode_arbitration_isStuck))begin
            _zz_172_ <= (3'b100);
          end
        end
        3'b100 : begin
          _zz_172_ <= (3'b000);
        end
        default : begin
        end
      endcase
      if(execute_CsrPlugin_csr_768)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
          CsrPlugin_mstatus_MPIE <= _zz_404_[0];
          CsrPlugin_mstatus_MIE <= _zz_405_[0];
          CsrPlugin_sstatus_SPP <= execute_CsrPlugin_writeData[8 : 8];
          CsrPlugin_sstatus_SPIE <= _zz_406_[0];
          CsrPlugin_sstatus_SIE <= _zz_407_[0];
          MmuPlugin_status_mxr <= _zz_408_[0];
          MmuPlugin_status_sum <= _zz_409_[0];
          MmuPlugin_status_mprv <= _zz_410_[0];
        end
      end
      if(execute_CsrPlugin_csr_836)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sip_STIP <= _zz_412_[0];
          CsrPlugin_sip_SSIP <= _zz_413_[0];
          CsrPlugin_sip_SEIP_SOFT <= _zz_414_[0];
        end
      end
      if(execute_CsrPlugin_csr_772)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mie_MEIE <= _zz_415_[0];
          CsrPlugin_mie_MTIE <= _zz_416_[0];
          CsrPlugin_mie_MSIE <= _zz_417_[0];
          CsrPlugin_sie_SEIE <= _zz_418_[0];
          CsrPlugin_sie_STIE <= _zz_419_[0];
          CsrPlugin_sie_SSIE <= _zz_420_[0];
        end
      end
      if(execute_CsrPlugin_csr_773)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mtvec_base <= execute_CsrPlugin_writeData[31 : 2];
          CsrPlugin_mtvec_mode <= execute_CsrPlugin_writeData[1 : 0];
        end
      end
      if(execute_CsrPlugin_csr_770)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_medeleg_EU <= _zz_421_[0];
          CsrPlugin_medeleg_II <= _zz_422_[0];
          CsrPlugin_medeleg_LAF <= _zz_423_[0];
          CsrPlugin_medeleg_LPF <= _zz_424_[0];
          CsrPlugin_medeleg_LAM <= _zz_425_[0];
          CsrPlugin_medeleg_SAF <= _zz_426_[0];
          CsrPlugin_medeleg_IAF <= _zz_427_[0];
          CsrPlugin_medeleg_ES <= _zz_428_[0];
          CsrPlugin_medeleg_IPF <= _zz_429_[0];
          CsrPlugin_medeleg_SPF <= _zz_430_[0];
          CsrPlugin_medeleg_SAM <= _zz_431_[0];
          CsrPlugin_medeleg_IAM <= _zz_432_[0];
        end
      end
      if(execute_CsrPlugin_csr_771)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mideleg_SE <= _zz_433_[0];
          CsrPlugin_mideleg_ST <= _zz_434_[0];
          CsrPlugin_mideleg_SS <= _zz_435_[0];
        end
      end
      if(execute_CsrPlugin_csr_256)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sstatus_SPP <= execute_CsrPlugin_writeData[8 : 8];
          CsrPlugin_sstatus_SPIE <= _zz_436_[0];
          CsrPlugin_sstatus_SIE <= _zz_437_[0];
          MmuPlugin_status_mxr <= _zz_438_[0];
          MmuPlugin_status_sum <= _zz_439_[0];
          MmuPlugin_status_mprv <= _zz_440_[0];
        end
      end
      if(execute_CsrPlugin_csr_324)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sip_STIP <= _zz_441_[0];
          CsrPlugin_sip_SSIP <= _zz_442_[0];
          CsrPlugin_sip_SEIP_SOFT <= _zz_443_[0];
        end
      end
      if(execute_CsrPlugin_csr_260)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sie_SEIE <= _zz_444_[0];
          CsrPlugin_sie_STIE <= _zz_445_[0];
          CsrPlugin_sie_SSIE <= _zz_446_[0];
        end
      end
      if(execute_CsrPlugin_csr_384)begin
        if(execute_CsrPlugin_writeEnable)begin
          MmuPlugin_satp_mode <= _zz_448_[0];
        end
      end
      if(iBus_cmd_ready)begin
        iBus_cmd_m2sPipe_rValid <= iBus_cmd_valid;
      end
      if(_zz_280_)begin
        dBus_cmd_halfPipe_regs_valid <= dBus_cmd_valid;
        dBus_cmd_halfPipe_regs_ready <= (! dBus_cmd_valid);
      end else begin
        dBus_cmd_halfPipe_regs_valid <= (! dBus_cmd_halfPipe_ready);
        dBus_cmd_halfPipe_regs_ready <= dBus_cmd_halfPipe_ready;
      end
    end
  end

  always @ (posedge clk) begin
    if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
      _zz_61_ <= IBusSimplePlugin_iBusRsp_stages_0_output_payload;
    end
    if(_zz_258_)begin
      IBusSimplePlugin_decompressor_bufferData <= IBusSimplePlugin_decompressor_input_payload_rsp_inst[31 : 16];
    end
    if(IBusSimplePlugin_decompressor_output_ready)begin
      _zz_90_ <= IBusSimplePlugin_decompressor_output_payload_pc;
      _zz_91_ <= IBusSimplePlugin_decompressor_output_payload_rsp_error;
      _zz_92_ <= IBusSimplePlugin_decompressor_output_payload_rsp_inst;
      _zz_93_ <= IBusSimplePlugin_decompressor_output_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_decompressor_raw;
    end
    if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
      IBusSimplePlugin_predictor_writeLast_valid <= IBusSimplePlugin_predictor_historyWriteDelayPatched_valid;
      IBusSimplePlugin_predictor_writeLast_payload_address <= IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_address;
      IBusSimplePlugin_predictor_writeLast_payload_data_source <= IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_source;
      IBusSimplePlugin_predictor_writeLast_payload_data_branchWish <= IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_branchWish;
      IBusSimplePlugin_predictor_writeLast_payload_data_last2Bytes <= IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_last2Bytes;
      IBusSimplePlugin_predictor_writeLast_payload_data_target <= IBusSimplePlugin_predictor_historyWriteDelayPatched_payload_data_target;
    end
    if(IBusSimplePlugin_iBusRsp_stages_0_input_ready)begin
      IBusSimplePlugin_predictor_buffer_pcCorrected <= IBusSimplePlugin_fetchPc_corrected;
    end
    if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
      IBusSimplePlugin_predictor_line_source <= IBusSimplePlugin_predictor_buffer_line_source;
      IBusSimplePlugin_predictor_line_branchWish <= IBusSimplePlugin_predictor_buffer_line_branchWish;
      IBusSimplePlugin_predictor_line_last2Bytes <= IBusSimplePlugin_predictor_buffer_line_last2Bytes;
      IBusSimplePlugin_predictor_line_target <= IBusSimplePlugin_predictor_buffer_line_target;
    end
    if(IBusSimplePlugin_iBusRsp_stages_0_output_ready)begin
      IBusSimplePlugin_predictor_buffer_hazard_regNextWhen <= IBusSimplePlugin_predictor_buffer_hazard;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_hazard <= IBusSimplePlugin_predictor_iBusRspContextOutput_hazard;
      IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_hit <= IBusSimplePlugin_predictor_iBusRspContextOutput_hit;
      IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_source <= IBusSimplePlugin_predictor_iBusRspContextOutput_line_source;
      IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_branchWish <= IBusSimplePlugin_predictor_iBusRspContextOutput_line_branchWish;
      IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_last2Bytes <= IBusSimplePlugin_predictor_iBusRspContextOutput_line_last2Bytes;
      IBusSimplePlugin_predictor_iBusRspContextOutput_delay_1_line_target <= IBusSimplePlugin_predictor_iBusRspContextOutput_line_target;
    end
    if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
      IBusSimplePlugin_mmu_joinCtx_physicalAddress <= IBusSimplePlugin_mmuBus_rsp_physicalAddress;
      IBusSimplePlugin_mmu_joinCtx_isIoAccess <= IBusSimplePlugin_mmuBus_rsp_isIoAccess;
      IBusSimplePlugin_mmu_joinCtx_isPaging <= IBusSimplePlugin_mmuBus_rsp_isPaging;
      IBusSimplePlugin_mmu_joinCtx_allowRead <= IBusSimplePlugin_mmuBus_rsp_allowRead;
      IBusSimplePlugin_mmu_joinCtx_allowWrite <= IBusSimplePlugin_mmuBus_rsp_allowWrite;
      IBusSimplePlugin_mmu_joinCtx_allowExecute <= IBusSimplePlugin_mmuBus_rsp_allowExecute;
      IBusSimplePlugin_mmu_joinCtx_exception <= IBusSimplePlugin_mmuBus_rsp_exception;
      IBusSimplePlugin_mmu_joinCtx_refilling <= IBusSimplePlugin_mmuBus_rsp_refilling;
    end
    `ifndef SYNTHESIS
      `ifdef FORMAL
        assert((! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck)))
      `else
        if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
          $display("FAILURE DBusSimplePlugin doesn't allow memory stage stall when read happend");
          $finish;
        end
      `endif
    `endif
    `ifndef SYNTHESIS
      `ifdef FORMAL
        assert((! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_MEMORY_STORE)) && writeBack_arbitration_isStuck)))
      `else
        if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_MEMORY_STORE)) && writeBack_arbitration_isStuck))) begin
          $display("FAILURE DBusSimplePlugin doesn't allow writeback stage stall when read happend");
          $finish;
        end
      `endif
    `endif
    _zz_132_ <= _zz_37_[11 : 7];
    _zz_133_ <= _zz_47_;
    if((memory_MulDivIterativePlugin_div_counter_value == 6'h20))begin
      memory_MulDivIterativePlugin_div_done <= 1'b1;
    end
    if((! memory_arbitration_isStuck))begin
      memory_MulDivIterativePlugin_div_done <= 1'b0;
    end
    if(_zz_227_)begin
      if(_zz_249_)begin
        memory_MulDivIterativePlugin_rs1[31 : 0] <= memory_MulDivIterativePlugin_div_stage_0_outNumerator;
        memory_MulDivIterativePlugin_accumulator[31 : 0] <= memory_MulDivIterativePlugin_div_stage_0_outRemainder;
        if((memory_MulDivIterativePlugin_div_counter_value == 6'h20))begin
          memory_MulDivIterativePlugin_div_result <= _zz_378_[31:0];
        end
      end
    end
    if(_zz_250_)begin
      memory_MulDivIterativePlugin_accumulator <= 65'h0;
      memory_MulDivIterativePlugin_rs1 <= ((_zz_143_ ? (~ _zz_144_) : _zz_144_) + _zz_384_);
      memory_MulDivIterativePlugin_rs2 <= ((_zz_142_ ? (~ execute_RS2) : execute_RS2) + _zz_386_);
      memory_MulDivIterativePlugin_div_needRevert <= ((_zz_143_ ^ (_zz_142_ && (! execute_INSTRUCTION[13]))) && (! (((execute_RS2 == 32'h0) && execute_IS_RS2_SIGNED) && (! execute_INSTRUCTION[13]))));
    end
    CsrPlugin_mip_MEIP <= externalInterrupt;
    CsrPlugin_mip_MTIP <= timerInterrupt;
    CsrPlugin_mip_MSIP <= softwareInterrupt;
    CsrPlugin_sip_SEIP_INPUT <= externalInterruptS;
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + 64'h0000000000000001);
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + 64'h0000000000000001);
    end
    if(_zz_228_)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= (_zz_153_ ? IBusSimplePlugin_decodeExceptionPort_payload_code : decodeExceptionPort_payload_code);
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= (_zz_153_ ? IBusSimplePlugin_decodeExceptionPort_payload_badAddr : decodeExceptionPort_payload_badAddr);
    end
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= CsrPlugin_selfException_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= CsrPlugin_selfException_payload_badAddr;
    end
    if(DBusSimplePlugin_memoryExceptionPort_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= DBusSimplePlugin_memoryExceptionPort_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= DBusSimplePlugin_memoryExceptionPort_payload_badAddr;
    end
    if(_zz_259_)begin
      if(_zz_260_)begin
        CsrPlugin_interrupt_code <= (4'b0101);
        CsrPlugin_interrupt_targetPrivilege <= (2'b01);
      end
      if(_zz_261_)begin
        CsrPlugin_interrupt_code <= (4'b0001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b01);
      end
      if(_zz_262_)begin
        CsrPlugin_interrupt_code <= (4'b1001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b01);
      end
    end
    if(_zz_263_)begin
      if(_zz_264_)begin
        CsrPlugin_interrupt_code <= (4'b0101);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_265_)begin
        CsrPlugin_interrupt_code <= (4'b0001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_266_)begin
        CsrPlugin_interrupt_code <= (4'b1001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_267_)begin
        CsrPlugin_interrupt_code <= (4'b0111);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_268_)begin
        CsrPlugin_interrupt_code <= (4'b0011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_269_)begin
        CsrPlugin_interrupt_code <= (4'b1011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
    end
    if(_zz_232_)begin
      case(CsrPlugin_targetPrivilege)
        2'b01 : begin
          CsrPlugin_scause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_scause_exceptionCode <= CsrPlugin_trapCause;
          CsrPlugin_sepc <= writeBack_PC;
          if(CsrPlugin_hadException)begin
            CsrPlugin_stval <= CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
          end
        end
        2'b11 : begin
          CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
          CsrPlugin_mepc <= writeBack_PC;
          if(CsrPlugin_hadException)begin
            CsrPlugin_mtval <= CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
          end
        end
        default : begin
        end
      endcase
    end
    if((MmuPlugin_dBusAccess_rsp_valid && (! MmuPlugin_dBusAccess_rsp_payload_redo)))begin
      MmuPlugin_shared_pteBuffer_V <= MmuPlugin_shared_dBusRsp_pte_V;
      MmuPlugin_shared_pteBuffer_R <= MmuPlugin_shared_dBusRsp_pte_R;
      MmuPlugin_shared_pteBuffer_W <= MmuPlugin_shared_dBusRsp_pte_W;
      MmuPlugin_shared_pteBuffer_X <= MmuPlugin_shared_dBusRsp_pte_X;
      MmuPlugin_shared_pteBuffer_U <= MmuPlugin_shared_dBusRsp_pte_U;
      MmuPlugin_shared_pteBuffer_G <= MmuPlugin_shared_dBusRsp_pte_G;
      MmuPlugin_shared_pteBuffer_A <= MmuPlugin_shared_dBusRsp_pte_A;
      MmuPlugin_shared_pteBuffer_D <= MmuPlugin_shared_dBusRsp_pte_D;
      MmuPlugin_shared_pteBuffer_RSW <= MmuPlugin_shared_dBusRsp_pte_RSW;
      MmuPlugin_shared_pteBuffer_PPN0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
      MmuPlugin_shared_pteBuffer_PPN1 <= MmuPlugin_shared_dBusRsp_pte_PPN1;
    end
    case(MmuPlugin_shared_state_1_)
      `MmuPlugin_shared_State_defaultEncoding_IDLE : begin
        if(_zz_270_)begin
          MmuPlugin_shared_vpn_1 <= IBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22];
          MmuPlugin_shared_vpn_0 <= IBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12];
          MmuPlugin_shared_portId <= (1'b0);
        end
        if(_zz_271_)begin
          MmuPlugin_shared_vpn_1 <= DBusSimplePlugin_mmuBus_cmd_virtualAddress[31 : 22];
          MmuPlugin_shared_vpn_0 <= DBusSimplePlugin_mmuBus_cmd_virtualAddress[21 : 12];
          MmuPlugin_shared_portId <= (1'b1);
        end
      end
      `MmuPlugin_shared_State_defaultEncoding_L1_CMD : begin
      end
      `MmuPlugin_shared_State_defaultEncoding_L1_RSP : begin
      end
      `MmuPlugin_shared_State_defaultEncoding_L0_CMD : begin
      end
      default : begin
      end
    endcase
    if(_zz_255_)begin
      if(_zz_256_)begin
        if(_zz_272_)begin
          MmuPlugin_ports_0_cache_0_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_0_cache_0_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_0_cache_0_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_0_cache_0_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_0_cache_0_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_0_cache_0_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_0_cache_0_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_0_cache_0_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_0_cache_0_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_0_cache_0_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
        if(_zz_273_)begin
          MmuPlugin_ports_0_cache_1_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_0_cache_1_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_0_cache_1_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_0_cache_1_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_0_cache_1_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_0_cache_1_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_0_cache_1_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_0_cache_1_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_0_cache_1_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_0_cache_1_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
        if(_zz_274_)begin
          MmuPlugin_ports_0_cache_2_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_0_cache_2_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_0_cache_2_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_0_cache_2_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_0_cache_2_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_0_cache_2_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_0_cache_2_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_0_cache_2_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_0_cache_2_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_0_cache_2_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
        if(_zz_275_)begin
          MmuPlugin_ports_0_cache_3_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_0_cache_3_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_0_cache_3_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_0_cache_3_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_0_cache_3_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_0_cache_3_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_0_cache_3_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_0_cache_3_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_0_cache_3_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_0_cache_3_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
      end
      if(_zz_257_)begin
        if(_zz_276_)begin
          MmuPlugin_ports_1_cache_0_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_1_cache_0_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_1_cache_0_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_1_cache_0_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_1_cache_0_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_1_cache_0_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_1_cache_0_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_1_cache_0_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_1_cache_0_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_1_cache_0_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
        if(_zz_277_)begin
          MmuPlugin_ports_1_cache_1_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_1_cache_1_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_1_cache_1_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_1_cache_1_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_1_cache_1_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_1_cache_1_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_1_cache_1_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_1_cache_1_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_1_cache_1_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_1_cache_1_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
        if(_zz_278_)begin
          MmuPlugin_ports_1_cache_2_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_1_cache_2_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_1_cache_2_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_1_cache_2_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_1_cache_2_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_1_cache_2_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_1_cache_2_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_1_cache_2_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_1_cache_2_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_1_cache_2_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
        if(_zz_279_)begin
          MmuPlugin_ports_1_cache_3_exception <= (MmuPlugin_shared_dBusRsp_exception || ((MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP) && (MmuPlugin_shared_dBusRsp_pte_PPN0 != 10'h0)));
          MmuPlugin_ports_1_cache_3_virtualAddress_0 <= MmuPlugin_shared_vpn_0;
          MmuPlugin_ports_1_cache_3_virtualAddress_1 <= MmuPlugin_shared_vpn_1;
          MmuPlugin_ports_1_cache_3_physicalAddress_0 <= MmuPlugin_shared_dBusRsp_pte_PPN0;
          MmuPlugin_ports_1_cache_3_physicalAddress_1 <= MmuPlugin_shared_dBusRsp_pte_PPN1[9 : 0];
          MmuPlugin_ports_1_cache_3_allowRead <= MmuPlugin_shared_dBusRsp_pte_R;
          MmuPlugin_ports_1_cache_3_allowWrite <= MmuPlugin_shared_dBusRsp_pte_W;
          MmuPlugin_ports_1_cache_3_allowExecute <= MmuPlugin_shared_dBusRsp_pte_X;
          MmuPlugin_ports_1_cache_3_allowUser <= MmuPlugin_shared_dBusRsp_pte_U;
          MmuPlugin_ports_1_cache_3_superPage <= (MmuPlugin_shared_state_1_ == `MmuPlugin_shared_State_defaultEncoding_L1_RSP);
        end
      end
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_LL <= execute_MUL_LL;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_DIV <= decode_IS_DIV;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_DIV <= execute_IS_DIV;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MMU_RSP_physicalAddress <= execute_MMU_RSP_physicalAddress;
      execute_to_memory_MMU_RSP_isIoAccess <= execute_MMU_RSP_isIoAccess;
      execute_to_memory_MMU_RSP_isPaging <= execute_MMU_RSP_isPaging;
      execute_to_memory_MMU_RSP_allowRead <= execute_MMU_RSP_allowRead;
      execute_to_memory_MMU_RSP_allowWrite <= execute_MMU_RSP_allowWrite;
      execute_to_memory_MMU_RSP_allowExecute <= execute_MMU_RSP_allowExecute;
      execute_to_memory_MMU_RSP_exception <= execute_MMU_RSP_exception;
      execute_to_memory_MMU_RSP_refilling <= execute_MMU_RSP_refilling;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_TARGET_MISSMATCH2 <= execute_TARGET_MISSMATCH2;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_31_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RS2_SIGNED <= decode_IS_RS2_SIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RS1_SIGNED <= decode_IS_RS1_SIGNED;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_HL <= execute_MUL_HL;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ALIGNEMENT_FAULT <= execute_ALIGNEMENT_FAULT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_RIGHT <= execute_SHIFT_RIGHT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_NEXT_PC2 <= execute_NEXT_PC2;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_30_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if(((! writeBack_arbitration_isStuck) && (! CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack)))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MUL_LOW <= memory_MUL_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_20_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_17_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_15_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ATOMIC <= decode_MEMORY_ATOMIC;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ATOMIC <= execute_MEMORY_ATOMIC;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ATOMIC <= memory_MEMORY_ATOMIC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_33_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_13_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PREDICTION_CONTEXT_hazard <= decode_PREDICTION_CONTEXT_hazard;
      decode_to_execute_PREDICTION_CONTEXT_hit <= decode_PREDICTION_CONTEXT_hit;
      decode_to_execute_PREDICTION_CONTEXT_line_source <= decode_PREDICTION_CONTEXT_line_source;
      decode_to_execute_PREDICTION_CONTEXT_line_branchWish <= decode_PREDICTION_CONTEXT_line_branchWish;
      decode_to_execute_PREDICTION_CONTEXT_line_last2Bytes <= decode_PREDICTION_CONTEXT_line_last2Bytes;
      decode_to_execute_PREDICTION_CONTEXT_line_target <= decode_PREDICTION_CONTEXT_line_target;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PREDICTION_CONTEXT_hazard <= execute_PREDICTION_CONTEXT_hazard;
      execute_to_memory_PREDICTION_CONTEXT_hit <= execute_PREDICTION_CONTEXT_hit;
      execute_to_memory_PREDICTION_CONTEXT_line_source <= execute_PREDICTION_CONTEXT_line_source;
      execute_to_memory_PREDICTION_CONTEXT_line_branchWish <= execute_PREDICTION_CONTEXT_line_branchWish;
      execute_to_memory_PREDICTION_CONTEXT_line_last2Bytes <= execute_PREDICTION_CONTEXT_line_last2Bytes;
      execute_to_memory_PREDICTION_CONTEXT_line_target <= execute_PREDICTION_CONTEXT_line_target;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_MUL <= decode_IS_MUL;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_MUL <= execute_IS_MUL;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_IS_MUL <= memory_IS_MUL;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_HH <= execute_MUL_HH;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MUL_HH <= memory_MUL_HH;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_LH <= execute_MUL_LH;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RVC <= decode_IS_RVC;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_RVC <= execute_IS_RVC;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MMU_FAULT <= execute_MMU_FAULT;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_SFENCE_VMA <= decode_IS_SFENCE_VMA;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_SFENCE_VMA <= execute_IS_SFENCE_VMA;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_IS_SFENCE_VMA <= memory_IS_SFENCE_VMA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_10_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= _zz_49_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_50_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_26_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_7_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_CTRL <= _zz_4_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_2_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ATOMIC_HIT <= execute_ATOMIC_HIT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ATOMIC_HIT <= memory_ATOMIC_HIT;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_DO_EBREAK <= decode_DO_EBREAK;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
    end
    if((_zz_172_ != (3'b000)))begin
      _zz_92_ <= IBusSimplePlugin_injectionPort_payload;
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_3857 <= (decode_INSTRUCTION[31 : 20] == 12'hf11);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_3858 <= (decode_INSTRUCTION[31 : 20] == 12'hf12);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_3859 <= (decode_INSTRUCTION[31 : 20] == 12'hf13);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_3860 <= (decode_INSTRUCTION[31 : 20] == 12'hf14);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_768 <= (decode_INSTRUCTION[31 : 20] == 12'h300);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_836 <= (decode_INSTRUCTION[31 : 20] == 12'h344);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_772 <= (decode_INSTRUCTION[31 : 20] == 12'h304);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_773 <= (decode_INSTRUCTION[31 : 20] == 12'h305);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_833 <= (decode_INSTRUCTION[31 : 20] == 12'h341);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_832 <= (decode_INSTRUCTION[31 : 20] == 12'h340);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_834 <= (decode_INSTRUCTION[31 : 20] == 12'h342);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_835 <= (decode_INSTRUCTION[31 : 20] == 12'h343);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_770 <= (decode_INSTRUCTION[31 : 20] == 12'h302);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_771 <= (decode_INSTRUCTION[31 : 20] == 12'h303);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_256 <= (decode_INSTRUCTION[31 : 20] == 12'h100);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_324 <= (decode_INSTRUCTION[31 : 20] == 12'h144);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_260 <= (decode_INSTRUCTION[31 : 20] == 12'h104);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_261 <= (decode_INSTRUCTION[31 : 20] == 12'h105);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_321 <= (decode_INSTRUCTION[31 : 20] == 12'h141);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_320 <= (decode_INSTRUCTION[31 : 20] == 12'h140);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_322 <= (decode_INSTRUCTION[31 : 20] == 12'h142);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_323 <= (decode_INSTRUCTION[31 : 20] == 12'h143);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_384 <= (decode_INSTRUCTION[31 : 20] == 12'h180);
    end
    if(execute_CsrPlugin_csr_836)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_mip_MSIP <= _zz_411_[0];
      end
    end
    if(execute_CsrPlugin_csr_833)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_mepc <= execute_CsrPlugin_writeData[31 : 0];
      end
    end
    if(execute_CsrPlugin_csr_832)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_mscratch <= execute_CsrPlugin_writeData[31 : 0];
      end
    end
    if(execute_CsrPlugin_csr_261)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_stvec_base <= execute_CsrPlugin_writeData[31 : 2];
        CsrPlugin_stvec_mode <= execute_CsrPlugin_writeData[1 : 0];
      end
    end
    if(execute_CsrPlugin_csr_321)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_sepc <= execute_CsrPlugin_writeData[31 : 0];
      end
    end
    if(execute_CsrPlugin_csr_320)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_sscratch <= execute_CsrPlugin_writeData[31 : 0];
      end
    end
    if(execute_CsrPlugin_csr_322)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_scause_interrupt <= _zz_447_[0];
        CsrPlugin_scause_exceptionCode <= execute_CsrPlugin_writeData[3 : 0];
      end
    end
    if(execute_CsrPlugin_csr_323)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_stval <= execute_CsrPlugin_writeData[31 : 0];
      end
    end
    if(execute_CsrPlugin_csr_384)begin
      if(execute_CsrPlugin_writeEnable)begin
        MmuPlugin_satp_ppn <= execute_CsrPlugin_writeData[19 : 0];
      end
    end
    if(iBus_cmd_ready)begin
      iBus_cmd_m2sPipe_rData_pc <= iBus_cmd_payload_pc;
    end
    if(_zz_280_)begin
      dBus_cmd_halfPipe_regs_payload_wr <= dBus_cmd_payload_wr;
      dBus_cmd_halfPipe_regs_payload_address <= dBus_cmd_payload_address;
      dBus_cmd_halfPipe_regs_payload_data <= dBus_cmd_payload_data;
      dBus_cmd_halfPipe_regs_payload_size <= dBus_cmd_payload_size;
    end
  end

  always @ (posedge clk) begin
    DebugPlugin_firstCycle <= 1'b0;
    if(debug_bus_cmd_ready)begin
      DebugPlugin_firstCycle <= 1'b1;
    end
    DebugPlugin_secondCycle <= DebugPlugin_firstCycle;
    DebugPlugin_isPipBusy <= (({writeBack_arbitration_isValid,{memory_arbitration_isValid,{execute_arbitration_isValid,decode_arbitration_isValid}}} != (4'b0000)) || IBusSimplePlugin_incomingInstruction);
    if(writeBack_arbitration_isValid)begin
      DebugPlugin_busReadDataReg <= _zz_47_;
    end
    _zz_154_ <= debug_bus_cmd_payload_address[2];
    if(_zz_230_)begin
      DebugPlugin_busReadDataReg <= execute_PC;
    end
    DebugPlugin_resetIt_regNext <= DebugPlugin_resetIt;
  end

  always @ (posedge clk or posedge debugReset) begin
    if (debugReset) begin
      DebugPlugin_resetIt <= 1'b0;
      DebugPlugin_haltIt <= 1'b0;
      DebugPlugin_stepIt <= 1'b0;
      DebugPlugin_godmode <= 1'b0;
      DebugPlugin_haltedByBreak <= 1'b0;
      _zz_155_ <= 1'b0;
      _zz_193_ <= 1'b0;
    end else begin
      if((DebugPlugin_haltIt && (! DebugPlugin_isPipBusy)))begin
        DebugPlugin_godmode <= 1'b1;
      end
      if(debug_bus_cmd_valid)begin
        case(_zz_254_)
          6'b000000 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_stepIt <= debug_bus_cmd_payload_data[4];
              if(debug_bus_cmd_payload_data[16])begin
                DebugPlugin_resetIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[24])begin
                DebugPlugin_resetIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[17])begin
                DebugPlugin_haltIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltedByBreak <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_godmode <= 1'b0;
              end
            end
          end
          6'b000001 : begin
          end
          default : begin
          end
        endcase
      end
      if(_zz_230_)begin
        if(_zz_231_)begin
          DebugPlugin_haltIt <= 1'b1;
          DebugPlugin_haltedByBreak <= 1'b1;
        end
      end
      if(_zz_234_)begin
        if(decode_arbitration_isValid)begin
          DebugPlugin_haltIt <= 1'b1;
        end
      end
      _zz_155_ <= (DebugPlugin_stepIt && decode_arbitration_isFiring);
      _zz_193_ <= (debug_bus_cmd_valid && debug_bus_cmd_ready);
    end
  end


endmodule
