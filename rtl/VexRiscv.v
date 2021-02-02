// Generator : SpinalHDL v1.4.0    git head : ecb5a80b713566f417ea3ea061f9969e73770a7f
// Date      : 28/01/2021, 15:40:44
// Component : VexRiscv


`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10

`define EnvCtrlEnum_defaultEncoding_type [1:0]
`define EnvCtrlEnum_defaultEncoding_NONE 2'b00
`define EnvCtrlEnum_defaultEncoding_XRET 2'b01
`define EnvCtrlEnum_defaultEncoding_WFI 2'b10
`define EnvCtrlEnum_defaultEncoding_ECALL 2'b11

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

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

module VexRiscv (
  input               timerInterrupt,
  input               externalInterrupt,
  input               softwareInterrupt,
  input               externalInterruptS,
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
  input               clk,
  input               reset 
);
  wire                _zz_183_;
  wire                _zz_184_;
  reg        [31:0]   _zz_185_;
  reg        [31:0]   _zz_186_;
  reg        [31:0]   _zz_187_;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  wire       [0:0]    IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy;
  wire                _zz_188_;
  wire                _zz_189_;
  wire                _zz_190_;
  wire                _zz_191_;
  wire                _zz_192_;
  wire                _zz_193_;
  wire                _zz_194_;
  wire                _zz_195_;
  wire                _zz_196_;
  wire                _zz_197_;
  wire       [1:0]    _zz_198_;
  wire                _zz_199_;
  wire                _zz_200_;
  wire                _zz_201_;
  wire                _zz_202_;
  wire                _zz_203_;
  wire                _zz_204_;
  wire                _zz_205_;
  wire                _zz_206_;
  wire       [1:0]    _zz_207_;
  wire                _zz_208_;
  wire                _zz_209_;
  wire                _zz_210_;
  wire                _zz_211_;
  wire                _zz_212_;
  wire                _zz_213_;
  wire                _zz_214_;
  wire                _zz_215_;
  wire                _zz_216_;
  wire                _zz_217_;
  wire                _zz_218_;
  wire                _zz_219_;
  wire                _zz_220_;
  wire                _zz_221_;
  wire                _zz_222_;
  wire                _zz_223_;
  wire                _zz_224_;
  wire                _zz_225_;
  wire       [4:0]    _zz_226_;
  wire       [1:0]    _zz_227_;
  wire       [1:0]    _zz_228_;
  wire       [1:0]    _zz_229_;
  wire       [1:0]    _zz_230_;
  wire                _zz_231_;
  wire       [0:0]    _zz_232_;
  wire       [32:0]   _zz_233_;
  wire       [31:0]   _zz_234_;
  wire       [32:0]   _zz_235_;
  wire       [0:0]    _zz_236_;
  wire       [2:0]    _zz_237_;
  wire       [31:0]   _zz_238_;
  wire       [0:0]    _zz_239_;
  wire       [0:0]    _zz_240_;
  wire       [0:0]    _zz_241_;
  wire       [0:0]    _zz_242_;
  wire       [0:0]    _zz_243_;
  wire       [0:0]    _zz_244_;
  wire       [0:0]    _zz_245_;
  wire       [0:0]    _zz_246_;
  wire       [51:0]   _zz_247_;
  wire       [51:0]   _zz_248_;
  wire       [51:0]   _zz_249_;
  wire       [32:0]   _zz_250_;
  wire       [51:0]   _zz_251_;
  wire       [49:0]   _zz_252_;
  wire       [51:0]   _zz_253_;
  wire       [49:0]   _zz_254_;
  wire       [51:0]   _zz_255_;
  wire       [0:0]    _zz_256_;
  wire       [0:0]    _zz_257_;
  wire       [0:0]    _zz_258_;
  wire       [0:0]    _zz_259_;
  wire       [0:0]    _zz_260_;
  wire       [3:0]    _zz_261_;
  wire       [2:0]    _zz_262_;
  wire       [31:0]   _zz_263_;
  wire       [2:0]    _zz_264_;
  wire       [31:0]   _zz_265_;
  wire       [31:0]   _zz_266_;
  wire       [11:0]   _zz_267_;
  wire       [11:0]   _zz_268_;
  wire       [11:0]   _zz_269_;
  wire       [31:0]   _zz_270_;
  wire       [19:0]   _zz_271_;
  wire       [11:0]   _zz_272_;
  wire       [2:0]    _zz_273_;
  wire       [0:0]    _zz_274_;
  wire       [2:0]    _zz_275_;
  wire       [0:0]    _zz_276_;
  wire       [2:0]    _zz_277_;
  wire       [0:0]    _zz_278_;
  wire       [2:0]    _zz_279_;
  wire       [0:0]    _zz_280_;
  wire       [2:0]    _zz_281_;
  wire       [0:0]    _zz_282_;
  wire       [2:0]    _zz_283_;
  wire       [4:0]    _zz_284_;
  wire       [11:0]   _zz_285_;
  wire       [11:0]   _zz_286_;
  wire       [31:0]   _zz_287_;
  wire       [31:0]   _zz_288_;
  wire       [31:0]   _zz_289_;
  wire       [31:0]   _zz_290_;
  wire       [31:0]   _zz_291_;
  wire       [31:0]   _zz_292_;
  wire       [31:0]   _zz_293_;
  wire       [65:0]   _zz_294_;
  wire       [65:0]   _zz_295_;
  wire       [31:0]   _zz_296_;
  wire       [31:0]   _zz_297_;
  wire       [0:0]    _zz_298_;
  wire       [5:0]    _zz_299_;
  wire       [32:0]   _zz_300_;
  wire       [31:0]   _zz_301_;
  wire       [31:0]   _zz_302_;
  wire       [32:0]   _zz_303_;
  wire       [32:0]   _zz_304_;
  wire       [32:0]   _zz_305_;
  wire       [32:0]   _zz_306_;
  wire       [0:0]    _zz_307_;
  wire       [32:0]   _zz_308_;
  wire       [0:0]    _zz_309_;
  wire       [32:0]   _zz_310_;
  wire       [0:0]    _zz_311_;
  wire       [31:0]   _zz_312_;
  wire       [1:0]    _zz_313_;
  wire       [1:0]    _zz_314_;
  wire       [11:0]   _zz_315_;
  wire       [19:0]   _zz_316_;
  wire       [11:0]   _zz_317_;
  wire       [2:0]    _zz_318_;
  wire       [0:0]    _zz_319_;
  wire       [0:0]    _zz_320_;
  wire       [0:0]    _zz_321_;
  wire       [0:0]    _zz_322_;
  wire       [0:0]    _zz_323_;
  wire       [0:0]    _zz_324_;
  wire       [0:0]    _zz_325_;
  wire       [0:0]    _zz_326_;
  wire       [0:0]    _zz_327_;
  wire       [0:0]    _zz_328_;
  wire       [0:0]    _zz_329_;
  wire       [0:0]    _zz_330_;
  wire       [0:0]    _zz_331_;
  wire       [0:0]    _zz_332_;
  wire       [0:0]    _zz_333_;
  wire       [0:0]    _zz_334_;
  wire       [0:0]    _zz_335_;
  wire       [0:0]    _zz_336_;
  wire       [0:0]    _zz_337_;
  wire       [0:0]    _zz_338_;
  wire       [0:0]    _zz_339_;
  wire       [0:0]    _zz_340_;
  wire       [0:0]    _zz_341_;
  wire       [0:0]    _zz_342_;
  wire       [0:0]    _zz_343_;
  wire       [0:0]    _zz_344_;
  wire       [0:0]    _zz_345_;
  wire       [0:0]    _zz_346_;
  wire       [0:0]    _zz_347_;
  wire       [0:0]    _zz_348_;
  wire       [0:0]    _zz_349_;
  wire       [0:0]    _zz_350_;
  wire       [0:0]    _zz_351_;
  wire       [0:0]    _zz_352_;
  wire       [0:0]    _zz_353_;
  wire       [0:0]    _zz_354_;
  wire       [0:0]    _zz_355_;
  wire       [0:0]    _zz_356_;
  wire                _zz_357_;
  wire                _zz_358_;
  wire       [1:0]    _zz_359_;
  wire       [31:0]   _zz_360_;
  wire       [31:0]   _zz_361_;
  wire       [31:0]   _zz_362_;
  wire                _zz_363_;
  wire       [0:0]    _zz_364_;
  wire       [12:0]   _zz_365_;
  wire       [31:0]   _zz_366_;
  wire       [31:0]   _zz_367_;
  wire       [31:0]   _zz_368_;
  wire                _zz_369_;
  wire       [0:0]    _zz_370_;
  wire       [6:0]    _zz_371_;
  wire       [31:0]   _zz_372_;
  wire       [31:0]   _zz_373_;
  wire       [31:0]   _zz_374_;
  wire                _zz_375_;
  wire       [0:0]    _zz_376_;
  wire       [0:0]    _zz_377_;
  wire                _zz_378_;
  wire                _zz_379_;
  wire       [6:0]    _zz_380_;
  wire       [4:0]    _zz_381_;
  wire                _zz_382_;
  wire       [4:0]    _zz_383_;
  wire       [0:0]    _zz_384_;
  wire       [7:0]    _zz_385_;
  wire                _zz_386_;
  wire       [0:0]    _zz_387_;
  wire       [0:0]    _zz_388_;
  wire       [31:0]   _zz_389_;
  wire       [31:0]   _zz_390_;
  wire       [31:0]   _zz_391_;
  wire       [31:0]   _zz_392_;
  wire                _zz_393_;
  wire       [0:0]    _zz_394_;
  wire       [0:0]    _zz_395_;
  wire       [0:0]    _zz_396_;
  wire       [0:0]    _zz_397_;
  wire       [1:0]    _zz_398_;
  wire       [1:0]    _zz_399_;
  wire                _zz_400_;
  wire       [0:0]    _zz_401_;
  wire       [23:0]   _zz_402_;
  wire       [31:0]   _zz_403_;
  wire       [31:0]   _zz_404_;
  wire       [31:0]   _zz_405_;
  wire       [31:0]   _zz_406_;
  wire       [31:0]   _zz_407_;
  wire       [31:0]   _zz_408_;
  wire       [31:0]   _zz_409_;
  wire                _zz_410_;
  wire       [1:0]    _zz_411_;
  wire       [1:0]    _zz_412_;
  wire                _zz_413_;
  wire       [0:0]    _zz_414_;
  wire       [20:0]   _zz_415_;
  wire       [31:0]   _zz_416_;
  wire       [31:0]   _zz_417_;
  wire       [31:0]   _zz_418_;
  wire       [31:0]   _zz_419_;
  wire                _zz_420_;
  wire                _zz_421_;
  wire                _zz_422_;
  wire       [3:0]    _zz_423_;
  wire       [3:0]    _zz_424_;
  wire                _zz_425_;
  wire       [0:0]    _zz_426_;
  wire       [17:0]   _zz_427_;
  wire       [31:0]   _zz_428_;
  wire       [31:0]   _zz_429_;
  wire                _zz_430_;
  wire       [0:0]    _zz_431_;
  wire       [0:0]    _zz_432_;
  wire       [31:0]   _zz_433_;
  wire       [31:0]   _zz_434_;
  wire                _zz_435_;
  wire       [1:0]    _zz_436_;
  wire       [1:0]    _zz_437_;
  wire                _zz_438_;
  wire       [0:0]    _zz_439_;
  wire       [14:0]   _zz_440_;
  wire       [31:0]   _zz_441_;
  wire       [31:0]   _zz_442_;
  wire       [31:0]   _zz_443_;
  wire                _zz_444_;
  wire       [0:0]    _zz_445_;
  wire       [3:0]    _zz_446_;
  wire       [0:0]    _zz_447_;
  wire       [0:0]    _zz_448_;
  wire                _zz_449_;
  wire       [0:0]    _zz_450_;
  wire       [11:0]   _zz_451_;
  wire       [31:0]   _zz_452_;
  wire                _zz_453_;
  wire       [0:0]    _zz_454_;
  wire       [0:0]    _zz_455_;
  wire       [31:0]   _zz_456_;
  wire       [0:0]    _zz_457_;
  wire       [4:0]    _zz_458_;
  wire       [0:0]    _zz_459_;
  wire       [0:0]    _zz_460_;
  wire                _zz_461_;
  wire       [0:0]    _zz_462_;
  wire       [8:0]    _zz_463_;
  wire       [31:0]   _zz_464_;
  wire       [31:0]   _zz_465_;
  wire       [31:0]   _zz_466_;
  wire       [31:0]   _zz_467_;
  wire                _zz_468_;
  wire       [0:0]    _zz_469_;
  wire       [1:0]    _zz_470_;
  wire       [31:0]   _zz_471_;
  wire       [31:0]   _zz_472_;
  wire       [31:0]   _zz_473_;
  wire       [0:0]    _zz_474_;
  wire       [1:0]    _zz_475_;
  wire       [1:0]    _zz_476_;
  wire       [1:0]    _zz_477_;
  wire                _zz_478_;
  wire       [0:0]    _zz_479_;
  wire       [5:0]    _zz_480_;
  wire       [31:0]   _zz_481_;
  wire       [31:0]   _zz_482_;
  wire       [31:0]   _zz_483_;
  wire                _zz_484_;
  wire                _zz_485_;
  wire       [31:0]   _zz_486_;
  wire       [31:0]   _zz_487_;
  wire                _zz_488_;
  wire                _zz_489_;
  wire                _zz_490_;
  wire                _zz_491_;
  wire       [0:0]    _zz_492_;
  wire       [0:0]    _zz_493_;
  wire                _zz_494_;
  wire       [0:0]    _zz_495_;
  wire       [3:0]    _zz_496_;
  wire       [31:0]   _zz_497_;
  wire       [31:0]   _zz_498_;
  wire       [31:0]   _zz_499_;
  wire       [31:0]   _zz_500_;
  wire       [31:0]   _zz_501_;
  wire       [31:0]   _zz_502_;
  wire       [0:0]    _zz_503_;
  wire       [3:0]    _zz_504_;
  wire       [0:0]    _zz_505_;
  wire       [0:0]    _zz_506_;
  wire                _zz_507_;
  wire       [0:0]    _zz_508_;
  wire       [1:0]    _zz_509_;
  wire       [31:0]   _zz_510_;
  wire       [31:0]   _zz_511_;
  wire       [31:0]   _zz_512_;
  wire                _zz_513_;
  wire                _zz_514_;
  wire       [31:0]   _zz_515_;
  wire       [31:0]   _zz_516_;
  wire       [31:0]   _zz_517_;
  wire       [31:0]   _zz_518_;
  wire                _zz_519_;
  wire       [0:0]    _zz_520_;
  wire       [1:0]    _zz_521_;
  wire                _zz_522_;
  wire                _zz_523_;
  wire       [31:0]   _zz_524_;
  wire       [31:0]   _zz_525_;
  wire                _zz_526_;
  wire                _zz_527_;
  wire                _zz_528_;
  wire       [31:0]   _zz_529_;
  wire       [31:0]   memory_PC;
  wire                decode_IS_RS2_SIGNED;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_1_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_2_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_3_;
  wire       [31:0]   execute_SHIFT_RIGHT;
  wire                decode_BYPASSABLE_EXECUTE_STAGE;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_4_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_5_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_6_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_7_;
  wire       `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_8_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_9_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_10_;
  wire                decode_CSR_WRITE_OPCODE;
  wire       [31:0]   writeBack_FORMAL_PC_NEXT;
  wire       [31:0]   memory_FORMAL_PC_NEXT;
  wire       [31:0]   execute_FORMAL_PC_NEXT;
  wire       [31:0]   decode_FORMAL_PC_NEXT;
  wire       [1:0]    memory_MEMORY_ADDRESS_LOW;
  wire       [1:0]    execute_MEMORY_ADDRESS_LOW;
  wire                execute_BRANCH_DO;
  wire                execute_BYPASSABLE_MEMORY_STAGE;
  wire                decode_BYPASSABLE_MEMORY_STAGE;
  wire                decode_IS_RS1_SIGNED;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_11_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_12_;
  wire       `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_13_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_14_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_15_;
  wire                decode_IS_DIV;
  wire       [31:0]   decode_SRC1;
  wire       [33:0]   memory_MUL_HH;
  wire       [33:0]   execute_MUL_HH;
  wire                memory_IS_MUL;
  wire                execute_IS_MUL;
  wire                decode_IS_MUL;
  wire       [31:0]   writeBack_REGFILE_WRITE_DATA;
  wire       [31:0]   memory_REGFILE_WRITE_DATA;
  wire       [31:0]   execute_REGFILE_WRITE_DATA;
  wire                decode_PREDICTION_HAD_BRANCHED2;
  wire       [31:0]   memory_MEMORY_READ_DATA;
  wire                decode_IS_CSR;
  wire       `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_16_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_17_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_18_;
  wire       [31:0]   decode_SRC2;
  wire       [33:0]   execute_MUL_HL;
  wire                decode_SRC_LESS_UNSIGNED;
  wire       [31:0]   execute_BRANCH_CALC;
  wire                decode_MEMORY_STORE;
  wire                decode_MEMORY_ENABLE;
  wire       [33:0]   execute_MUL_LH;
  wire                decode_SRC2_FORCE_ZERO;
  wire                decode_CSR_READ_OPCODE;
  wire       [51:0]   memory_MUL_LOW;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_19_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_20_;
  wire       [31:0]   execute_MUL_LL;
  wire       [31:0]   memory_BRANCH_CALC;
  wire                memory_BRANCH_DO;
  wire                execute_IS_RVC;
  wire       [31:0]   execute_PC;
  wire                execute_BRANCH_COND_RESULT;
  wire                execute_PREDICTION_HAD_BRANCHED2;
  wire       `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_21_;
  wire                execute_CSR_READ_OPCODE;
  wire                execute_CSR_WRITE_OPCODE;
  wire                execute_IS_CSR;
  wire       `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_22_;
  wire       `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_23_;
  wire       `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_24_;
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
  (* syn_keep , keep *) wire       [31:0]   execute_RS1 /* synthesis syn_keep = 1 */ ;
  wire                decode_RS2_USE;
  wire                decode_RS1_USE;
  reg        [31:0]   _zz_25_;
  wire                execute_REGFILE_WRITE_VALID;
  wire                execute_BYPASSABLE_EXECUTE_STAGE;
  wire                memory_REGFILE_WRITE_VALID;
  wire       [31:0]   memory_INSTRUCTION;
  wire                memory_BYPASSABLE_MEMORY_STAGE;
  wire                writeBack_REGFILE_WRITE_VALID;
  reg        [31:0]   decode_RS2;
  reg        [31:0]   decode_RS1;
  wire       [31:0]   memory_SHIFT_RIGHT;
  reg        [31:0]   _zz_26_;
  wire       `ShiftCtrlEnum_defaultEncoding_type memory_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_27_;
  wire       `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_28_;
  wire                execute_SRC_LESS_UNSIGNED;
  wire                execute_SRC2_FORCE_ZERO;
  wire                execute_SRC_USE_SUB_LESS;
  wire       [31:0]   _zz_29_;
  wire       [31:0]   _zz_30_;
  wire       `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_31_;
  wire       [31:0]   _zz_32_;
  wire       `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_33_;
  wire                decode_SRC_USE_SUB_LESS;
  wire                decode_SRC_ADD_ZERO;
  wire       [31:0]   execute_SRC_ADD_SUB;
  wire                execute_SRC_LESS;
  wire       `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_34_;
  wire       [31:0]   execute_SRC2;
  wire       [31:0]   execute_SRC1;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_35_;
  wire       [31:0]   _zz_36_;
  wire                _zz_37_;
  reg                 _zz_38_;
  wire       [31:0]   decode_INSTRUCTION_ANTICIPATED;
  reg                 decode_REGFILE_WRITE_VALID;
  wire                decode_LEGAL_INSTRUCTION;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_39_;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_40_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_41_;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_42_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_43_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_44_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_45_;
  wire                writeBack_MEMORY_STORE;
  reg        [31:0]   _zz_46_;
  wire                writeBack_MEMORY_ENABLE;
  wire       [1:0]    writeBack_MEMORY_ADDRESS_LOW;
  wire       [31:0]   writeBack_MEMORY_READ_DATA;
  wire                memory_MEMORY_STORE;
  wire                memory_MEMORY_ENABLE;
  wire       [31:0]   execute_SRC_ADD;
  (* syn_keep , keep *) wire       [31:0]   execute_RS2 /* synthesis syn_keep = 1 */ ;
  wire       [31:0]   execute_INSTRUCTION;
  wire                execute_MEMORY_STORE;
  wire                execute_MEMORY_ENABLE;
  wire                execute_ALIGNEMENT_FAULT;
  wire       `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_47_;
  reg        [31:0]   _zz_48_;
  reg        [31:0]   _zz_49_;
  reg        [31:0]   _zz_50_;
  wire       [31:0]   decode_PC;
  wire       [31:0]   decode_INSTRUCTION;
  wire                decode_IS_RVC;
  wire       [31:0]   writeBack_PC;
  wire       [31:0]   writeBack_INSTRUCTION;
  wire                decode_arbitration_haltItself;
  reg                 decode_arbitration_haltByOther;
  reg                 decode_arbitration_removeIt;
  wire                decode_arbitration_flushIt;
  reg                 decode_arbitration_flushNext;
  wire                decode_arbitration_isValid;
  wire                decode_arbitration_isStuck;
  wire                decode_arbitration_isStuckByOthers;
  wire                decode_arbitration_isFlushed;
  wire                decode_arbitration_isMoving;
  wire                decode_arbitration_isFiring;
  reg                 execute_arbitration_haltItself;
  wire                execute_arbitration_haltByOther;
  reg                 execute_arbitration_removeIt;
  wire                execute_arbitration_flushIt;
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
  wire                memory_arbitration_flushIt;
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
  wire                IBusSimplePlugin_predictionJumpInterface_valid;
  (* syn_keep , keep *) wire       [31:0]   IBusSimplePlugin_predictionJumpInterface_payload /* synthesis syn_keep = 1 */ ;
  wire                IBusSimplePlugin_decodePrediction_cmd_hadBranch;
  wire                IBusSimplePlugin_decodePrediction_rsp_wasWrong;
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
  wire                decodeExceptionPort_valid;
  wire       [3:0]    decodeExceptionPort_payload_code;
  wire       [31:0]   decodeExceptionPort_payload_badAddr;
  reg                 CsrPlugin_inWfi /* verilator public */ ;
  wire                CsrPlugin_thirdPartyWake;
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
  wire                CsrPlugin_forceMachineWire;
  reg                 CsrPlugin_selfException_valid;
  reg        [3:0]    CsrPlugin_selfException_payload_code;
  wire       [31:0]   CsrPlugin_selfException_payload_badAddr;
  wire                CsrPlugin_allowInterrupts;
  wire                CsrPlugin_allowException;
  wire                BranchPlugin_jumpInterface_valid;
  wire       [31:0]   BranchPlugin_jumpInterface_payload;
  wire                IBusSimplePlugin_externalFlush;
  wire                IBusSimplePlugin_jump_pcLoad_valid;
  wire       [31:0]   IBusSimplePlugin_jump_pcLoad_payload;
  wire       [3:0]    _zz_51_;
  wire       [3:0]    _zz_52_;
  wire                _zz_53_;
  wire                _zz_54_;
  wire                _zz_55_;
  wire                IBusSimplePlugin_fetchPc_output_valid;
  wire                IBusSimplePlugin_fetchPc_output_ready;
  wire       [31:0]   IBusSimplePlugin_fetchPc_output_payload;
  reg        [31:0]   IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg                 IBusSimplePlugin_fetchPc_correction;
  reg                 IBusSimplePlugin_fetchPc_correctionReg;
  wire                IBusSimplePlugin_fetchPc_corrected;
  reg                 IBusSimplePlugin_fetchPc_pcRegPropagate;
  reg                 IBusSimplePlugin_fetchPc_booted;
  reg                 IBusSimplePlugin_fetchPc_inc;
  reg        [31:0]   IBusSimplePlugin_fetchPc_pc;
  reg                 IBusSimplePlugin_fetchPc_flushed;
  reg                 IBusSimplePlugin_decodePc_flushed;
  reg        [31:0]   IBusSimplePlugin_decodePc_pcReg /* verilator public */ ;
  wire       [31:0]   IBusSimplePlugin_decodePc_pcPlus;
  wire                IBusSimplePlugin_decodePc_injectedDecode;
  wire                IBusSimplePlugin_iBusRsp_redoFetch;
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
  wire                _zz_56_;
  wire                _zz_57_;
  wire                IBusSimplePlugin_iBusRsp_flush;
  wire                _zz_58_;
  wire                _zz_59_;
  reg                 _zz_60_;
  reg                 IBusSimplePlugin_iBusRsp_readyForError;
  wire                IBusSimplePlugin_iBusRsp_output_valid;
  wire                IBusSimplePlugin_iBusRsp_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_output_payload_pc;
  wire                IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
  wire                IBusSimplePlugin_iBusRsp_output_payload_isRvc;
  wire                IBusSimplePlugin_decompressor_input_valid;
  wire                IBusSimplePlugin_decompressor_input_ready;
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
  wire       [15:0]   _zz_61_;
  reg        [31:0]   IBusSimplePlugin_decompressor_decompressed;
  wire       [4:0]    _zz_62_;
  wire       [4:0]    _zz_63_;
  wire       [11:0]   _zz_64_;
  wire                _zz_65_;
  reg        [11:0]   _zz_66_;
  wire                _zz_67_;
  reg        [9:0]    _zz_68_;
  wire       [20:0]   _zz_69_;
  wire                _zz_70_;
  reg        [14:0]   _zz_71_;
  wire                _zz_72_;
  reg        [2:0]    _zz_73_;
  wire                _zz_74_;
  reg        [9:0]    _zz_75_;
  wire       [20:0]   _zz_76_;
  wire                _zz_77_;
  reg        [4:0]    _zz_78_;
  wire       [12:0]   _zz_79_;
  wire       [4:0]    _zz_80_;
  wire       [4:0]    _zz_81_;
  wire       [4:0]    _zz_82_;
  wire                _zz_83_;
  reg        [2:0]    _zz_84_;
  reg        [2:0]    _zz_85_;
  wire                _zz_86_;
  reg        [6:0]    _zz_87_;
  wire                IBusSimplePlugin_decompressor_bufferFill;
  wire                IBusSimplePlugin_injector_decodeInput_valid;
  wire                IBusSimplePlugin_injector_decodeInput_ready;
  wire       [31:0]   IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire                IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire                IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg                 _zz_88_;
  reg        [31:0]   _zz_89_;
  reg                 _zz_90_;
  reg        [31:0]   _zz_91_;
  reg                 _zz_92_;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_2;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_3;
  reg        [31:0]   IBusSimplePlugin_injector_formal_rawInDecode;
  wire                _zz_93_;
  reg        [18:0]   _zz_94_;
  wire                _zz_95_;
  reg        [10:0]   _zz_96_;
  wire                _zz_97_;
  reg        [18:0]   _zz_98_;
  wire                IBusSimplePlugin_cmd_valid;
  wire                IBusSimplePlugin_cmd_ready;
  wire       [31:0]   IBusSimplePlugin_cmd_payload_pc;
  wire                IBusSimplePlugin_pending_inc;
  wire                IBusSimplePlugin_pending_dec;
  reg        [2:0]    IBusSimplePlugin_pending_value;
  wire       [2:0]    IBusSimplePlugin_pending_next;
  wire                IBusSimplePlugin_cmdFork_canEmit;
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
  wire                _zz_99_;
  wire                dBus_cmd_valid;
  wire                dBus_cmd_ready;
  wire                dBus_cmd_payload_wr;
  wire       [31:0]   dBus_cmd_payload_address;
  wire       [31:0]   dBus_cmd_payload_data;
  wire       [1:0]    dBus_cmd_payload_size;
  wire                dBus_rsp_ready;
  wire                dBus_rsp_error;
  wire       [31:0]   dBus_rsp_data;
  wire                _zz_100_;
  reg                 execute_DBusSimplePlugin_skipCmd;
  reg        [31:0]   _zz_101_;
  reg        [3:0]    _zz_102_;
  wire       [3:0]    execute_DBusSimplePlugin_formalMask;
  reg        [31:0]   writeBack_DBusSimplePlugin_rspShifted;
  wire                _zz_103_;
  reg        [31:0]   _zz_104_;
  wire                _zz_105_;
  reg        [31:0]   _zz_106_;
  reg        [31:0]   writeBack_DBusSimplePlugin_rspFormated;
  wire       [29:0]   _zz_107_;
  wire                _zz_108_;
  wire                _zz_109_;
  wire                _zz_110_;
  wire                _zz_111_;
  wire                _zz_112_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_113_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_114_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_115_;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_116_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_117_;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_118_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_119_;
  wire       [4:0]    decode_RegFilePlugin_regFileReadAddress1;
  wire       [4:0]    decode_RegFilePlugin_regFileReadAddress2;
  wire       [31:0]   decode_RegFilePlugin_rs1Data;
  wire       [31:0]   decode_RegFilePlugin_rs2Data;
  reg                 lastStageRegFileWrite_valid /* verilator public */ ;
  wire       [4:0]    lastStageRegFileWrite_payload_address /* verilator public */ ;
  wire       [31:0]   lastStageRegFileWrite_payload_data /* verilator public */ ;
  reg                 _zz_120_;
  reg        [31:0]   execute_IntAluPlugin_bitwise;
  reg        [31:0]   _zz_121_;
  reg        [31:0]   _zz_122_;
  wire                _zz_123_;
  reg        [19:0]   _zz_124_;
  wire                _zz_125_;
  reg        [19:0]   _zz_126_;
  reg        [31:0]   _zz_127_;
  reg        [31:0]   execute_SrcPlugin_addSub;
  wire                execute_SrcPlugin_less;
  wire       [4:0]    execute_FullBarrelShifterPlugin_amplitude;
  reg        [31:0]   _zz_128_;
  wire       [31:0]   execute_FullBarrelShifterPlugin_reversed;
  reg        [31:0]   _zz_129_;
  reg                 _zz_130_;
  reg                 _zz_131_;
  reg                 _zz_132_;
  reg        [4:0]    _zz_133_;
  reg        [31:0]   _zz_134_;
  wire                _zz_135_;
  wire                _zz_136_;
  wire                _zz_137_;
  wire                _zz_138_;
  wire                _zz_139_;
  wire                _zz_140_;
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
  wire       [31:0]   _zz_141_;
  wire       [32:0]   memory_MulDivIterativePlugin_div_stage_0_remainderShifted;
  wire       [32:0]   memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator;
  wire       [31:0]   memory_MulDivIterativePlugin_div_stage_0_outRemainder;
  wire       [31:0]   memory_MulDivIterativePlugin_div_stage_0_outNumerator;
  wire       [31:0]   _zz_142_;
  wire                _zz_143_;
  wire                _zz_144_;
  reg        [32:0]   _zz_145_;
  reg        [1:0]    _zz_146_;
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
  wire                _zz_147_;
  wire                _zz_148_;
  wire                _zz_149_;
  wire                _zz_150_;
  wire                _zz_151_;
  wire                _zz_152_;
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
  wire       [1:0]    _zz_153_;
  wire                _zz_154_;
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
  wire                execute_BranchPlugin_eq;
  wire       [2:0]    _zz_155_;
  reg                 _zz_156_;
  reg                 _zz_157_;
  wire                execute_BranchPlugin_missAlignedTarget;
  reg        [31:0]   execute_BranchPlugin_branch_src1;
  reg        [31:0]   execute_BranchPlugin_branch_src2;
  wire                _zz_158_;
  reg        [19:0]   _zz_159_;
  wire                _zz_160_;
  reg        [10:0]   _zz_161_;
  wire                _zz_162_;
  reg        [18:0]   _zz_163_;
  wire       [31:0]   execute_BranchPlugin_branchAdder;
  reg        [31:0]   execute_to_memory_MUL_LL;
  reg        `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg        [51:0]   memory_to_writeBack_MUL_LOW;
  reg                 decode_to_execute_CSR_READ_OPCODE;
  reg                 decode_to_execute_SRC2_FORCE_ZERO;
  reg                 decode_to_execute_IS_RVC;
  reg        [33:0]   execute_to_memory_MUL_LH;
  reg                 decode_to_execute_MEMORY_ENABLE;
  reg                 execute_to_memory_MEMORY_ENABLE;
  reg                 memory_to_writeBack_MEMORY_ENABLE;
  reg                 decode_to_execute_MEMORY_STORE;
  reg                 execute_to_memory_MEMORY_STORE;
  reg                 memory_to_writeBack_MEMORY_STORE;
  reg        [31:0]   execute_to_memory_BRANCH_CALC;
  reg                 decode_to_execute_SRC_LESS_UNSIGNED;
  reg        [33:0]   execute_to_memory_MUL_HL;
  reg        [31:0]   decode_to_execute_SRC2;
  reg        `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg                 decode_to_execute_IS_CSR;
  reg        [31:0]   memory_to_writeBack_MEMORY_READ_DATA;
  reg                 decode_to_execute_PREDICTION_HAD_BRANCHED2;
  reg        [31:0]   execute_to_memory_REGFILE_WRITE_DATA;
  reg        [31:0]   memory_to_writeBack_REGFILE_WRITE_DATA;
  reg        [31:0]   decode_to_execute_INSTRUCTION;
  reg        [31:0]   execute_to_memory_INSTRUCTION;
  reg        [31:0]   memory_to_writeBack_INSTRUCTION;
  reg                 decode_to_execute_IS_MUL;
  reg                 execute_to_memory_IS_MUL;
  reg                 memory_to_writeBack_IS_MUL;
  reg                 decode_to_execute_REGFILE_WRITE_VALID;
  reg                 execute_to_memory_REGFILE_WRITE_VALID;
  reg                 memory_to_writeBack_REGFILE_WRITE_VALID;
  reg        [33:0]   execute_to_memory_MUL_HH;
  reg        [33:0]   memory_to_writeBack_MUL_HH;
  reg        [31:0]   decode_to_execute_SRC1;
  reg                 decode_to_execute_IS_DIV;
  reg                 execute_to_memory_IS_DIV;
  reg        `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg        `ShiftCtrlEnum_defaultEncoding_type execute_to_memory_SHIFT_CTRL;
  reg                 decode_to_execute_IS_RS1_SIGNED;
  reg                 decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg                 execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg        [31:0]   decode_to_execute_RS1;
  reg                 execute_to_memory_BRANCH_DO;
  reg        [1:0]    execute_to_memory_MEMORY_ADDRESS_LOW;
  reg        [1:0]    memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg        [31:0]   decode_to_execute_FORMAL_PC_NEXT;
  reg        [31:0]   execute_to_memory_FORMAL_PC_NEXT;
  reg        [31:0]   memory_to_writeBack_FORMAL_PC_NEXT;
  reg                 decode_to_execute_CSR_WRITE_OPCODE;
  reg        `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg        `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg        `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg                 decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg                 decode_to_execute_SRC_USE_SUB_LESS;
  reg        [31:0]   execute_to_memory_SHIFT_RIGHT;
  reg        [31:0]   decode_to_execute_RS2;
  reg        `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg                 decode_to_execute_IS_RS2_SIGNED;
  reg        [31:0]   decode_to_execute_PC;
  reg        [31:0]   execute_to_memory_PC;
  reg        [31:0]   memory_to_writeBack_PC;
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
  reg        [31:0]   _zz_164_;
  reg        [31:0]   _zz_165_;
  reg        [31:0]   _zz_166_;
  reg        [31:0]   _zz_167_;
  reg        [31:0]   _zz_168_;
  reg        [31:0]   _zz_169_;
  reg        [31:0]   _zz_170_;
  reg        [31:0]   _zz_171_;
  reg        [31:0]   _zz_172_;
  reg        [31:0]   _zz_173_;
  reg        [31:0]   _zz_174_;
  reg        [31:0]   _zz_175_;
  reg        [31:0]   _zz_176_;
  reg        [31:0]   _zz_177_;
  reg        [31:0]   _zz_178_;
  reg        [31:0]   _zz_179_;
  reg        [31:0]   _zz_180_;
  reg        [31:0]   _zz_181_;
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
  reg        [3:0]    _zz_182_;
  `ifndef SYNTHESIS
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_1__string;
  reg [39:0] _zz_2__string;
  reg [39:0] _zz_3__string;
  reg [39:0] _zz_4__string;
  reg [39:0] _zz_5__string;
  reg [39:0] _zz_6__string;
  reg [39:0] _zz_7__string;
  reg [39:0] decode_ENV_CTRL_string;
  reg [39:0] _zz_8__string;
  reg [39:0] _zz_9__string;
  reg [39:0] _zz_10__string;
  reg [71:0] _zz_11__string;
  reg [71:0] _zz_12__string;
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_13__string;
  reg [71:0] _zz_14__string;
  reg [71:0] _zz_15__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_16__string;
  reg [63:0] _zz_17__string;
  reg [63:0] _zz_18__string;
  reg [31:0] _zz_19__string;
  reg [31:0] _zz_20__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_21__string;
  reg [39:0] memory_ENV_CTRL_string;
  reg [39:0] _zz_22__string;
  reg [39:0] execute_ENV_CTRL_string;
  reg [39:0] _zz_23__string;
  reg [39:0] writeBack_ENV_CTRL_string;
  reg [39:0] _zz_24__string;
  reg [71:0] memory_SHIFT_CTRL_string;
  reg [71:0] _zz_27__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_28__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_31__string;
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_33__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_34__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_35__string;
  reg [71:0] _zz_39__string;
  reg [95:0] _zz_40__string;
  reg [39:0] _zz_41__string;
  reg [23:0] _zz_42__string;
  reg [63:0] _zz_43__string;
  reg [39:0] _zz_44__string;
  reg [31:0] _zz_45__string;
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_47__string;
  reg [31:0] _zz_113__string;
  reg [39:0] _zz_114__string;
  reg [63:0] _zz_115__string;
  reg [23:0] _zz_116__string;
  reg [39:0] _zz_117__string;
  reg [95:0] _zz_118__string;
  reg [71:0] _zz_119__string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  reg [71:0] execute_to_memory_SHIFT_CTRL_string;
  reg [39:0] decode_to_execute_ENV_CTRL_string;
  reg [39:0] execute_to_memory_ENV_CTRL_string;
  reg [39:0] memory_to_writeBack_ENV_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  `endif

  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;

  assign _zz_188_ = (execute_arbitration_isValid && execute_IS_CSR);
  assign _zz_189_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_190_ = 1'b1;
  assign _zz_191_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_192_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_193_ = (memory_arbitration_isValid && memory_IS_DIV);
  assign _zz_194_ = ({decodeExceptionPort_valid,IBusSimplePlugin_decodeExceptionPort_valid} != (2'b00));
  assign _zz_195_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_WFI));
  assign _zz_196_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_197_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_198_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_199_ = (IBusSimplePlugin_jump_pcLoad_valid && ((! decode_arbitration_isStuck) || decode_arbitration_removeIt));
  assign _zz_200_ = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_payload_rsp_error);
  assign _zz_201_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_202_ = (1'b0 || (! 1'b1));
  assign _zz_203_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_204_ = (1'b0 || (! memory_BYPASSABLE_MEMORY_STAGE));
  assign _zz_205_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_206_ = (1'b0 || (! execute_BYPASSABLE_EXECUTE_STAGE));
  assign _zz_207_ = execute_INSTRUCTION[13 : 12];
  assign _zz_208_ = (memory_MulDivIterativePlugin_frontendOk && (! memory_MulDivIterativePlugin_div_done));
  assign _zz_209_ = (! memory_arbitration_isStuck);
  assign _zz_210_ = (CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]);
  assign _zz_211_ = (execute_CsrPlugin_illegalAccess || execute_CsrPlugin_illegalInstruction);
  assign _zz_212_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_ECALL));
  assign _zz_213_ = (IBusSimplePlugin_decompressor_output_ready && IBusSimplePlugin_decompressor_input_valid);
  assign _zz_214_ = ((CsrPlugin_sstatus_SIE && (CsrPlugin_privilege == (2'b01))) || (CsrPlugin_privilege < (2'b01)));
  assign _zz_215_ = ((_zz_147_ && (1'b1 && CsrPlugin_mideleg_ST)) && (! 1'b0));
  assign _zz_216_ = ((_zz_148_ && (1'b1 && CsrPlugin_mideleg_SS)) && (! 1'b0));
  assign _zz_217_ = ((_zz_149_ && (1'b1 && CsrPlugin_mideleg_SE)) && (! 1'b0));
  assign _zz_218_ = (CsrPlugin_mstatus_MIE || (CsrPlugin_privilege < (2'b11)));
  assign _zz_219_ = ((_zz_147_ && 1'b1) && (! (CsrPlugin_mideleg_ST != (1'b0))));
  assign _zz_220_ = ((_zz_148_ && 1'b1) && (! (CsrPlugin_mideleg_SS != (1'b0))));
  assign _zz_221_ = ((_zz_149_ && 1'b1) && (! (CsrPlugin_mideleg_SE != (1'b0))));
  assign _zz_222_ = ((_zz_150_ && 1'b1) && (! 1'b0));
  assign _zz_223_ = ((_zz_151_ && 1'b1) && (! 1'b0));
  assign _zz_224_ = ((_zz_152_ && 1'b1) && (! 1'b0));
  assign _zz_225_ = (! dBus_cmd_halfPipe_regs_valid);
  assign _zz_226_ = {_zz_61_[1 : 0],_zz_61_[15 : 13]};
  assign _zz_227_ = _zz_61_[6 : 5];
  assign _zz_228_ = _zz_61_[11 : 10];
  assign _zz_229_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_230_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_231_ = execute_INSTRUCTION[13];
  assign _zz_232_ = _zz_107_[6 : 6];
  assign _zz_233_ = ($signed(_zz_235_) >>> execute_FullBarrelShifterPlugin_amplitude);
  assign _zz_234_ = _zz_233_[31 : 0];
  assign _zz_235_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_FullBarrelShifterPlugin_reversed[31]),execute_FullBarrelShifterPlugin_reversed};
  assign _zz_236_ = _zz_107_[5 : 5];
  assign _zz_237_ = (decode_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_238_ = {29'd0, _zz_237_};
  assign _zz_239_ = _zz_107_[15 : 15];
  assign _zz_240_ = _zz_107_[4 : 4];
  assign _zz_241_ = _zz_107_[0 : 0];
  assign _zz_242_ = _zz_107_[1 : 1];
  assign _zz_243_ = _zz_107_[3 : 3];
  assign _zz_244_ = _zz_107_[23 : 23];
  assign _zz_245_ = _zz_107_[21 : 21];
  assign _zz_246_ = _zz_107_[18 : 18];
  assign _zz_247_ = ($signed(_zz_248_) + $signed(_zz_253_));
  assign _zz_248_ = ($signed(_zz_249_) + $signed(_zz_251_));
  assign _zz_249_ = 52'h0;
  assign _zz_250_ = {1'b0,memory_MUL_LL};
  assign _zz_251_ = {{19{_zz_250_[32]}}, _zz_250_};
  assign _zz_252_ = ({16'd0,memory_MUL_LH} <<< 16);
  assign _zz_253_ = {{2{_zz_252_[49]}}, _zz_252_};
  assign _zz_254_ = ({16'd0,memory_MUL_HL} <<< 16);
  assign _zz_255_ = {{2{_zz_254_[49]}}, _zz_254_};
  assign _zz_256_ = _zz_107_[22 : 22];
  assign _zz_257_ = _zz_107_[20 : 20];
  assign _zz_258_ = _zz_107_[9 : 9];
  assign _zz_259_ = _zz_107_[19 : 19];
  assign _zz_260_ = _zz_107_[12 : 12];
  assign _zz_261_ = (_zz_51_ - (4'b0001));
  assign _zz_262_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_263_ = {29'd0, _zz_262_};
  assign _zz_264_ = (decode_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_265_ = {29'd0, _zz_264_};
  assign _zz_266_ = {{_zz_71_,_zz_61_[6 : 2]},12'h0};
  assign _zz_267_ = {{{(4'b0000),_zz_61_[8 : 7]},_zz_61_[12 : 9]},(2'b00)};
  assign _zz_268_ = {{{(4'b0000),_zz_61_[8 : 7]},_zz_61_[12 : 9]},(2'b00)};
  assign _zz_269_ = {{{decode_INSTRUCTION[31],decode_INSTRUCTION[7]},decode_INSTRUCTION[30 : 25]},decode_INSTRUCTION[11 : 8]};
  assign _zz_270_ = {{_zz_94_,{{{decode_INSTRUCTION[31],decode_INSTRUCTION[7]},decode_INSTRUCTION[30 : 25]},decode_INSTRUCTION[11 : 8]}},1'b0};
  assign _zz_271_ = {{{decode_INSTRUCTION[31],decode_INSTRUCTION[19 : 12]},decode_INSTRUCTION[20]},decode_INSTRUCTION[30 : 21]};
  assign _zz_272_ = {{{decode_INSTRUCTION[31],decode_INSTRUCTION[7]},decode_INSTRUCTION[30 : 25]},decode_INSTRUCTION[11 : 8]};
  assign _zz_273_ = (IBusSimplePlugin_pending_value + _zz_275_);
  assign _zz_274_ = IBusSimplePlugin_pending_inc;
  assign _zz_275_ = {2'd0, _zz_274_};
  assign _zz_276_ = IBusSimplePlugin_pending_dec;
  assign _zz_277_ = {2'd0, _zz_276_};
  assign _zz_278_ = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter != (3'b000)));
  assign _zz_279_ = {2'd0, _zz_278_};
  assign _zz_280_ = IBusSimplePlugin_pending_dec;
  assign _zz_281_ = {2'd0, _zz_280_};
  assign _zz_282_ = execute_SRC_LESS;
  assign _zz_283_ = (decode_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_284_ = decode_INSTRUCTION[19 : 15];
  assign _zz_285_ = decode_INSTRUCTION[31 : 20];
  assign _zz_286_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_287_ = ($signed(_zz_288_) + $signed(_zz_291_));
  assign _zz_288_ = ($signed(_zz_289_) + $signed(_zz_290_));
  assign _zz_289_ = execute_SRC1;
  assign _zz_290_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_291_ = (execute_SRC_USE_SUB_LESS ? _zz_292_ : _zz_293_);
  assign _zz_292_ = 32'h00000001;
  assign _zz_293_ = 32'h0;
  assign _zz_294_ = {{14{writeBack_MUL_LOW[51]}}, writeBack_MUL_LOW};
  assign _zz_295_ = ({32'd0,writeBack_MUL_HH} <<< 32);
  assign _zz_296_ = writeBack_MUL_LOW[31 : 0];
  assign _zz_297_ = writeBack_MulPlugin_result[63 : 32];
  assign _zz_298_ = memory_MulDivIterativePlugin_div_counter_willIncrement;
  assign _zz_299_ = {5'd0, _zz_298_};
  assign _zz_300_ = {1'd0, memory_MulDivIterativePlugin_rs2};
  assign _zz_301_ = memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator[31:0];
  assign _zz_302_ = memory_MulDivIterativePlugin_div_stage_0_remainderShifted[31:0];
  assign _zz_303_ = {_zz_141_,(! memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator[32])};
  assign _zz_304_ = _zz_305_;
  assign _zz_305_ = _zz_306_;
  assign _zz_306_ = ({1'b0,(memory_MulDivIterativePlugin_div_needRevert ? (~ _zz_142_) : _zz_142_)} + _zz_308_);
  assign _zz_307_ = memory_MulDivIterativePlugin_div_needRevert;
  assign _zz_308_ = {32'd0, _zz_307_};
  assign _zz_309_ = _zz_144_;
  assign _zz_310_ = {32'd0, _zz_309_};
  assign _zz_311_ = _zz_143_;
  assign _zz_312_ = {31'd0, _zz_311_};
  assign _zz_313_ = (_zz_153_ & (~ _zz_314_));
  assign _zz_314_ = (_zz_153_ - (2'b01));
  assign _zz_315_ = execute_INSTRUCTION[31 : 20];
  assign _zz_316_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_317_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_318_ = (execute_IS_RVC ? (3'b010) : (3'b100));
  assign _zz_319_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_320_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_321_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_322_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_323_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_324_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_325_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_326_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_327_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_328_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_329_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_330_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_331_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_332_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_333_ = execute_CsrPlugin_writeData[8 : 8];
  assign _zz_334_ = execute_CsrPlugin_writeData[2 : 2];
  assign _zz_335_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_336_ = execute_CsrPlugin_writeData[13 : 13];
  assign _zz_337_ = execute_CsrPlugin_writeData[4 : 4];
  assign _zz_338_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_339_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_340_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_341_ = execute_CsrPlugin_writeData[12 : 12];
  assign _zz_342_ = execute_CsrPlugin_writeData[15 : 15];
  assign _zz_343_ = execute_CsrPlugin_writeData[6 : 6];
  assign _zz_344_ = execute_CsrPlugin_writeData[0 : 0];
  assign _zz_345_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_346_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_347_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_348_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_349_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_350_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_351_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_352_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_353_ = execute_CsrPlugin_writeData[9 : 9];
  assign _zz_354_ = execute_CsrPlugin_writeData[5 : 5];
  assign _zz_355_ = execute_CsrPlugin_writeData[1 : 1];
  assign _zz_356_ = execute_CsrPlugin_writeData[31 : 31];
  assign _zz_357_ = 1'b1;
  assign _zz_358_ = 1'b1;
  assign _zz_359_ = {_zz_55_,_zz_54_};
  assign _zz_360_ = 32'h0000107f;
  assign _zz_361_ = (decode_INSTRUCTION & 32'h0000207f);
  assign _zz_362_ = 32'h00002073;
  assign _zz_363_ = ((decode_INSTRUCTION & 32'h0000407f) == 32'h00004063);
  assign _zz_364_ = ((decode_INSTRUCTION & 32'h0000207f) == 32'h00002013);
  assign _zz_365_ = {((decode_INSTRUCTION & 32'h0000603f) == 32'h00000023),{((decode_INSTRUCTION & 32'h0000207f) == 32'h00000003),{((decode_INSTRUCTION & _zz_366_) == 32'h00000003),{(_zz_367_ == _zz_368_),{_zz_369_,{_zz_370_,_zz_371_}}}}}};
  assign _zz_366_ = 32'h0000505f;
  assign _zz_367_ = (decode_INSTRUCTION & 32'h0000707b);
  assign _zz_368_ = 32'h00000063;
  assign _zz_369_ = ((decode_INSTRUCTION & 32'h0000607f) == 32'h0000000f);
  assign _zz_370_ = ((decode_INSTRUCTION & 32'hfc00007f) == 32'h00000033);
  assign _zz_371_ = {((decode_INSTRUCTION & 32'hbc00707f) == 32'h00005013),{((decode_INSTRUCTION & 32'hfc00307f) == 32'h00001013),{((decode_INSTRUCTION & _zz_372_) == 32'h00005033),{(_zz_373_ == _zz_374_),{_zz_375_,{_zz_376_,_zz_377_}}}}}};
  assign _zz_372_ = 32'hbe00707f;
  assign _zz_373_ = (decode_INSTRUCTION & 32'hbe00707f);
  assign _zz_374_ = 32'h00000033;
  assign _zz_375_ = ((decode_INSTRUCTION & 32'hdfffffff) == 32'h10200073);
  assign _zz_376_ = ((decode_INSTRUCTION & 32'hffffffff) == 32'h10500073);
  assign _zz_377_ = ((decode_INSTRUCTION & 32'hffffffff) == 32'h00000073);
  assign _zz_378_ = (_zz_61_[11 : 10] == (2'b01));
  assign _zz_379_ = ((_zz_61_[11 : 10] == (2'b11)) && (_zz_61_[6 : 5] == (2'b00)));
  assign _zz_380_ = 7'h0;
  assign _zz_381_ = _zz_61_[6 : 2];
  assign _zz_382_ = _zz_61_[12];
  assign _zz_383_ = _zz_61_[11 : 7];
  assign _zz_384_ = decode_INSTRUCTION[31];
  assign _zz_385_ = decode_INSTRUCTION[19 : 12];
  assign _zz_386_ = decode_INSTRUCTION[20];
  assign _zz_387_ = decode_INSTRUCTION[31];
  assign _zz_388_ = decode_INSTRUCTION[7];
  assign _zz_389_ = (decode_INSTRUCTION & 32'h00007034);
  assign _zz_390_ = 32'h00005010;
  assign _zz_391_ = (decode_INSTRUCTION & 32'h02007064);
  assign _zz_392_ = 32'h00005020;
  assign _zz_393_ = ((decode_INSTRUCTION & 32'h40003054) == 32'h40001010);
  assign _zz_394_ = ((decode_INSTRUCTION & _zz_403_) == 32'h00001010);
  assign _zz_395_ = ((decode_INSTRUCTION & _zz_404_) == 32'h00001010);
  assign _zz_396_ = ((decode_INSTRUCTION & _zz_405_) == 32'h00000004);
  assign _zz_397_ = _zz_112_;
  assign _zz_398_ = {(_zz_406_ == _zz_407_),_zz_112_};
  assign _zz_399_ = (2'b00);
  assign _zz_400_ = ((_zz_408_ == _zz_409_) != (1'b0));
  assign _zz_401_ = (_zz_410_ != (1'b0));
  assign _zz_402_ = {(_zz_411_ != _zz_412_),{_zz_413_,{_zz_414_,_zz_415_}}};
  assign _zz_403_ = 32'h00007034;
  assign _zz_404_ = 32'h02007054;
  assign _zz_405_ = 32'h00000014;
  assign _zz_406_ = (decode_INSTRUCTION & 32'h00000044);
  assign _zz_407_ = 32'h00000004;
  assign _zz_408_ = (decode_INSTRUCTION & 32'h00001000);
  assign _zz_409_ = 32'h00001000;
  assign _zz_410_ = ((decode_INSTRUCTION & 32'h00003000) == 32'h00002000);
  assign _zz_411_ = {(_zz_416_ == _zz_417_),(_zz_418_ == _zz_419_)};
  assign _zz_412_ = (2'b00);
  assign _zz_413_ = ({_zz_420_,_zz_421_} != (2'b00));
  assign _zz_414_ = (_zz_422_ != (1'b0));
  assign _zz_415_ = {(_zz_423_ != _zz_424_),{_zz_425_,{_zz_426_,_zz_427_}}};
  assign _zz_416_ = (decode_INSTRUCTION & 32'h00002010);
  assign _zz_417_ = 32'h00002000;
  assign _zz_418_ = (decode_INSTRUCTION & 32'h00005000);
  assign _zz_419_ = 32'h00001000;
  assign _zz_420_ = ((decode_INSTRUCTION & 32'h00000034) == 32'h00000020);
  assign _zz_421_ = ((decode_INSTRUCTION & 32'h00000064) == 32'h00000020);
  assign _zz_422_ = ((decode_INSTRUCTION & 32'h00000020) == 32'h00000020);
  assign _zz_423_ = {(_zz_428_ == _zz_429_),{_zz_430_,{_zz_431_,_zz_432_}}};
  assign _zz_424_ = (4'b0000);
  assign _zz_425_ = ((_zz_433_ == _zz_434_) != (1'b0));
  assign _zz_426_ = (_zz_435_ != (1'b0));
  assign _zz_427_ = {(_zz_436_ != _zz_437_),{_zz_438_,{_zz_439_,_zz_440_}}};
  assign _zz_428_ = (decode_INSTRUCTION & 32'h00000044);
  assign _zz_429_ = 32'h0;
  assign _zz_430_ = ((decode_INSTRUCTION & 32'h00000018) == 32'h0);
  assign _zz_431_ = _zz_111_;
  assign _zz_432_ = ((decode_INSTRUCTION & _zz_441_) == 32'h00001000);
  assign _zz_433_ = (decode_INSTRUCTION & 32'h00000064);
  assign _zz_434_ = 32'h00000024;
  assign _zz_435_ = ((decode_INSTRUCTION & 32'h00000058) == 32'h0);
  assign _zz_436_ = {_zz_109_,(_zz_442_ == _zz_443_)};
  assign _zz_437_ = (2'b00);
  assign _zz_438_ = ({_zz_109_,_zz_444_} != (2'b00));
  assign _zz_439_ = ({_zz_445_,_zz_446_} != 5'h0);
  assign _zz_440_ = {(_zz_447_ != _zz_448_),{_zz_449_,{_zz_450_,_zz_451_}}};
  assign _zz_441_ = 32'h00005004;
  assign _zz_442_ = (decode_INSTRUCTION & 32'h00000070);
  assign _zz_443_ = 32'h00000020;
  assign _zz_444_ = ((decode_INSTRUCTION & 32'h00000020) == 32'h0);
  assign _zz_445_ = ((decode_INSTRUCTION & _zz_452_) == 32'h00000040);
  assign _zz_446_ = {_zz_109_,{_zz_453_,{_zz_454_,_zz_455_}}};
  assign _zz_447_ = ((decode_INSTRUCTION & _zz_456_) == 32'h00004000);
  assign _zz_448_ = (1'b0);
  assign _zz_449_ = (_zz_111_ != (1'b0));
  assign _zz_450_ = ({_zz_457_,_zz_458_} != 6'h0);
  assign _zz_451_ = {(_zz_459_ != _zz_460_),{_zz_461_,{_zz_462_,_zz_463_}}};
  assign _zz_452_ = 32'h00000040;
  assign _zz_453_ = ((decode_INSTRUCTION & 32'h00004020) == 32'h00004020);
  assign _zz_454_ = ((decode_INSTRUCTION & _zz_464_) == 32'h00000010);
  assign _zz_455_ = ((decode_INSTRUCTION & _zz_465_) == 32'h00000020);
  assign _zz_456_ = 32'h00004004;
  assign _zz_457_ = _zz_110_;
  assign _zz_458_ = {(_zz_466_ == _zz_467_),{_zz_468_,{_zz_469_,_zz_470_}}};
  assign _zz_459_ = ((decode_INSTRUCTION & _zz_471_) == 32'h00000050);
  assign _zz_460_ = (1'b0);
  assign _zz_461_ = ((_zz_472_ == _zz_473_) != (1'b0));
  assign _zz_462_ = ({_zz_474_,_zz_475_} != (3'b000));
  assign _zz_463_ = {(_zz_476_ != _zz_477_),{_zz_478_,{_zz_479_,_zz_480_}}};
  assign _zz_464_ = 32'h00000030;
  assign _zz_465_ = 32'h02000020;
  assign _zz_466_ = (decode_INSTRUCTION & 32'h00001010);
  assign _zz_467_ = 32'h00001010;
  assign _zz_468_ = ((decode_INSTRUCTION & _zz_481_) == 32'h00002010);
  assign _zz_469_ = (_zz_482_ == _zz_483_);
  assign _zz_470_ = {_zz_484_,_zz_485_};
  assign _zz_471_ = 32'h00203050;
  assign _zz_472_ = (decode_INSTRUCTION & 32'h00403050);
  assign _zz_473_ = 32'h00000050;
  assign _zz_474_ = (_zz_486_ == _zz_487_);
  assign _zz_475_ = {_zz_488_,_zz_489_};
  assign _zz_476_ = {_zz_110_,_zz_490_};
  assign _zz_477_ = (2'b00);
  assign _zz_478_ = (_zz_491_ != (1'b0));
  assign _zz_479_ = (_zz_492_ != _zz_493_);
  assign _zz_480_ = {_zz_494_,{_zz_495_,_zz_496_}};
  assign _zz_481_ = 32'h00002010;
  assign _zz_482_ = (decode_INSTRUCTION & 32'h00000050);
  assign _zz_483_ = 32'h00000010;
  assign _zz_484_ = ((decode_INSTRUCTION & _zz_497_) == 32'h00000004);
  assign _zz_485_ = ((decode_INSTRUCTION & _zz_498_) == 32'h0);
  assign _zz_486_ = (decode_INSTRUCTION & 32'h00000044);
  assign _zz_487_ = 32'h00000040;
  assign _zz_488_ = ((decode_INSTRUCTION & _zz_499_) == 32'h00002010);
  assign _zz_489_ = ((decode_INSTRUCTION & _zz_500_) == 32'h40000030);
  assign _zz_490_ = ((decode_INSTRUCTION & _zz_501_) == 32'h00000004);
  assign _zz_491_ = ((decode_INSTRUCTION & _zz_502_) == 32'h00000040);
  assign _zz_492_ = _zz_108_;
  assign _zz_493_ = (1'b0);
  assign _zz_494_ = ({_zz_503_,_zz_504_} != 5'h0);
  assign _zz_495_ = (_zz_505_ != _zz_506_);
  assign _zz_496_ = {_zz_507_,{_zz_508_,_zz_509_}};
  assign _zz_497_ = 32'h0000000c;
  assign _zz_498_ = 32'h00000028;
  assign _zz_499_ = 32'h00002014;
  assign _zz_500_ = 32'h40000034;
  assign _zz_501_ = 32'h0000001c;
  assign _zz_502_ = 32'h00000058;
  assign _zz_503_ = _zz_109_;
  assign _zz_504_ = {((decode_INSTRUCTION & _zz_510_) == 32'h00002010),{(_zz_511_ == _zz_512_),{_zz_513_,_zz_514_}}};
  assign _zz_505_ = _zz_108_;
  assign _zz_506_ = (1'b0);
  assign _zz_507_ = ({(_zz_515_ == _zz_516_),(_zz_517_ == _zz_518_)} != (2'b00));
  assign _zz_508_ = ({_zz_519_,{_zz_520_,_zz_521_}} != (4'b0000));
  assign _zz_509_ = {(_zz_522_ != (1'b0)),(_zz_523_ != (1'b0))};
  assign _zz_510_ = 32'h00002030;
  assign _zz_511_ = (decode_INSTRUCTION & 32'h00001030);
  assign _zz_512_ = 32'h00000010;
  assign _zz_513_ = ((decode_INSTRUCTION & 32'h02002060) == 32'h00002020);
  assign _zz_514_ = ((decode_INSTRUCTION & 32'h02003020) == 32'h00000020);
  assign _zz_515_ = (decode_INSTRUCTION & 32'h00001050);
  assign _zz_516_ = 32'h00001050;
  assign _zz_517_ = (decode_INSTRUCTION & 32'h00002050);
  assign _zz_518_ = 32'h00002050;
  assign _zz_519_ = ((decode_INSTRUCTION & 32'h00002040) == 32'h00002040);
  assign _zz_520_ = ((decode_INSTRUCTION & 32'h00001040) == 32'h00001040);
  assign _zz_521_ = {((decode_INSTRUCTION & _zz_524_) == 32'h00000040),((decode_INSTRUCTION & _zz_525_) == 32'h00000040)};
  assign _zz_522_ = ((decode_INSTRUCTION & 32'h02004074) == 32'h02000030);
  assign _zz_523_ = ((decode_INSTRUCTION & 32'h02004064) == 32'h02004020);
  assign _zz_524_ = 32'h00000050;
  assign _zz_525_ = 32'h00400040;
  assign _zz_526_ = execute_INSTRUCTION[31];
  assign _zz_527_ = execute_INSTRUCTION[31];
  assign _zz_528_ = execute_INSTRUCTION[7];
  assign _zz_529_ = 32'h0;
  initial begin
    $readmemb("VexRiscv.v_toplevel_RegFilePlugin_regFile.bin",RegFilePlugin_regFile);
  end
  always @ (posedge clk) begin
    if(_zz_357_) begin
      _zz_185_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge clk) begin
    if(_zz_358_) begin
      _zz_186_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  always @ (posedge clk) begin
    if(_zz_38_) begin
      RegFilePlugin_regFile[lastStageRegFileWrite_payload_address] <= lastStageRegFileWrite_payload_data;
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid            (iBus_rsp_valid                                                  ), //i
    .io_push_ready            (IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready              ), //o
    .io_push_payload_error    (iBus_rsp_payload_error                                          ), //i
    .io_push_payload_inst     (iBus_rsp_payload_inst[31:0]                                     ), //i
    .io_pop_valid             (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid               ), //o
    .io_pop_ready             (_zz_183_                                                        ), //i
    .io_pop_payload_error     (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error       ), //o
    .io_pop_payload_inst      (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst[31:0]  ), //o
    .io_flush                 (_zz_184_                                                        ), //i
    .io_occupancy             (IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy               ), //o
    .clk                      (clk                                                             ), //i
    .reset                    (reset                                                           )  //i
  );
  always @(*) begin
    case(_zz_359_)
      2'b00 : begin
        _zz_187_ = CsrPlugin_jumpInterface_payload;
      end
      2'b01 : begin
        _zz_187_ = BranchPlugin_jumpInterface_payload;
      end
      2'b10 : begin
        _zz_187_ = CsrPlugin_redoInterface_payload;
      end
      default : begin
        _zz_187_ = IBusSimplePlugin_predictionJumpInterface_payload;
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_1__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_1__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_1__string = "AND_1";
      default : _zz_1__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_2_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_2__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_2__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_2__string = "AND_1";
      default : _zz_2__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_3__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_3__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_3__string = "AND_1";
      default : _zz_3__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_4_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_4__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_4__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_4__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_4__string = "ECALL";
      default : _zz_4__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_5__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_5__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_5__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_5__string = "ECALL";
      default : _zz_5__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_6_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_6__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_6__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_6__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_6__string = "ECALL";
      default : _zz_6__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_7_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_7__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_7__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_7__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_7__string = "ECALL";
      default : _zz_7__string = "?????";
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
    case(_zz_8_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_8__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_8__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_8__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_8__string = "ECALL";
      default : _zz_8__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_9__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_9__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_9__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_9__string = "ECALL";
      default : _zz_9__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_10_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_10__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_10__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_10__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_10__string = "ECALL";
      default : _zz_10__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_11_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_11__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_11__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_11__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_11__string = "SRA_1    ";
      default : _zz_11__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_12_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_12__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_12__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_12__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_12__string = "SRA_1    ";
      default : _zz_12__string = "?????????";
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
    case(_zz_13_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_13__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_13__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_13__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_13__string = "SRA_1    ";
      default : _zz_13__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_14_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_14__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_14__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_14__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_14__string = "SRA_1    ";
      default : _zz_14__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_15_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_15__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_15__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_15__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_15__string = "SRA_1    ";
      default : _zz_15__string = "?????????";
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
    case(_zz_16_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_16__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_16__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_16__string = "BITWISE ";
      default : _zz_16__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_17_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_17__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_17__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_17__string = "BITWISE ";
      default : _zz_17__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_18_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_18__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_18__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_18__string = "BITWISE ";
      default : _zz_18__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_19_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_19__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_19__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_19__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_19__string = "JALR";
      default : _zz_19__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_20_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_20__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_20__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_20__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_20__string = "JALR";
      default : _zz_20__string = "????";
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
    case(_zz_21_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_21__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_21__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_21__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_21__string = "JALR";
      default : _zz_21__string = "????";
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
    case(_zz_22_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_22__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_22__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_22__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_22__string = "ECALL";
      default : _zz_22__string = "?????";
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
    case(_zz_23_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_23__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_23__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_23__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_23__string = "ECALL";
      default : _zz_23__string = "?????";
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
    case(_zz_24_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_24__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_24__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_24__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_24__string = "ECALL";
      default : _zz_24__string = "?????";
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
    case(_zz_27_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_27__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_27__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_27__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_27__string = "SRA_1    ";
      default : _zz_27__string = "?????????";
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
    case(_zz_28_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_28__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_28__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_28__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_28__string = "SRA_1    ";
      default : _zz_28__string = "?????????";
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
    case(_zz_31_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_31__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_31__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_31__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_31__string = "PC ";
      default : _zz_31__string = "???";
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
    case(_zz_33_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_33__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_33__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_33__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_33__string = "URS1        ";
      default : _zz_33__string = "????????????";
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
    case(_zz_34_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_34__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_34__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_34__string = "BITWISE ";
      default : _zz_34__string = "????????";
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
    case(_zz_35_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_35__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_35__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_35__string = "AND_1";
      default : _zz_35__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_39_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_39__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_39__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_39__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_39__string = "SRA_1    ";
      default : _zz_39__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_40_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_40__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_40__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_40__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_40__string = "URS1        ";
      default : _zz_40__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_41_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_41__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_41__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_41__string = "AND_1";
      default : _zz_41__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_42_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_42__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_42__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_42__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_42__string = "PC ";
      default : _zz_42__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_43_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_43__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_43__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_43__string = "BITWISE ";
      default : _zz_43__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_44_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_44__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_44__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_44__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_44__string = "ECALL";
      default : _zz_44__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_45_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_45__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_45__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_45__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_45__string = "JALR";
      default : _zz_45__string = "????";
    endcase
  end
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
    case(_zz_47_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_47__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_47__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_47__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_47__string = "JALR";
      default : _zz_47__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_113_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_113__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_113__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_113__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_113__string = "JALR";
      default : _zz_113__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_114_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_114__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_114__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_WFI : _zz_114__string = "WFI  ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_114__string = "ECALL";
      default : _zz_114__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_115_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_115__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_115__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_115__string = "BITWISE ";
      default : _zz_115__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_116_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_116__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_116__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_116__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_116__string = "PC ";
      default : _zz_116__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_117_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_117__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_117__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_117__string = "AND_1";
      default : _zz_117__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_118_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_118__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_118__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_118__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_118__string = "URS1        ";
      default : _zz_118__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_119_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_119__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_119__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_119__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_119__string = "SRA_1    ";
      default : _zz_119__string = "?????????";
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
  `endif

  assign memory_PC = execute_to_memory_PC;
  assign decode_IS_RS2_SIGNED = _zz_232_[0];
  assign decode_ALU_BITWISE_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign execute_SHIFT_RIGHT = _zz_234_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_236_[0];
  assign _zz_4_ = _zz_5_;
  assign _zz_6_ = _zz_7_;
  assign decode_ENV_CTRL = _zz_8_;
  assign _zz_9_ = _zz_10_;
  assign decode_CSR_WRITE_OPCODE = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == 5'h0)) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == 5'h0))));
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = (decode_PC + _zz_238_);
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = dBus_cmd_payload_address[1 : 0];
  assign execute_BRANCH_DO = ((execute_PREDICTION_HAD_BRANCHED2 != execute_BRANCH_COND_RESULT) || execute_BranchPlugin_missAlignedTarget);
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_239_[0];
  assign decode_IS_RS1_SIGNED = _zz_240_[0];
  assign _zz_11_ = _zz_12_;
  assign decode_SHIFT_CTRL = _zz_13_;
  assign _zz_14_ = _zz_15_;
  assign decode_IS_DIV = _zz_241_[0];
  assign decode_SRC1 = _zz_122_;
  assign memory_MUL_HH = execute_to_memory_MUL_HH;
  assign execute_MUL_HH = ($signed(execute_MulPlugin_aHigh) * $signed(execute_MulPlugin_bHigh));
  assign memory_IS_MUL = execute_to_memory_IS_MUL;
  assign execute_IS_MUL = decode_to_execute_IS_MUL;
  assign decode_IS_MUL = _zz_242_[0];
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_121_;
  assign decode_PREDICTION_HAD_BRANCHED2 = IBusSimplePlugin_decodePrediction_cmd_hadBranch;
  assign memory_MEMORY_READ_DATA = dBus_rsp_data;
  assign decode_IS_CSR = _zz_243_[0];
  assign decode_ALU_CTRL = _zz_16_;
  assign _zz_17_ = _zz_18_;
  assign decode_SRC2 = _zz_127_;
  assign execute_MUL_HL = ($signed(execute_MulPlugin_aHigh) * $signed(execute_MulPlugin_bSLow));
  assign decode_SRC_LESS_UNSIGNED = _zz_244_[0];
  assign execute_BRANCH_CALC = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign decode_MEMORY_STORE = _zz_245_[0];
  assign decode_MEMORY_ENABLE = _zz_246_[0];
  assign execute_MUL_LH = ($signed(execute_MulPlugin_aSLow) * $signed(execute_MulPlugin_bHigh));
  assign decode_SRC2_FORCE_ZERO = (decode_SRC_ADD_ZERO && (! decode_SRC_USE_SUB_LESS));
  assign decode_CSR_READ_OPCODE = (decode_INSTRUCTION[13 : 7] != 7'h20);
  assign memory_MUL_LOW = ($signed(_zz_247_) + $signed(_zz_255_));
  assign _zz_19_ = _zz_20_;
  assign execute_MUL_LL = (execute_MulPlugin_aULow * execute_MulPlugin_bULow);
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_IS_RVC = decode_to_execute_IS_RVC;
  assign execute_PC = decode_to_execute_PC;
  assign execute_BRANCH_COND_RESULT = _zz_157_;
  assign execute_PREDICTION_HAD_BRANCHED2 = decode_to_execute_PREDICTION_HAD_BRANCHED2;
  assign execute_BRANCH_CTRL = _zz_21_;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_22_;
  assign execute_ENV_CTRL = _zz_23_;
  assign writeBack_ENV_CTRL = _zz_24_;
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
  assign decode_RS2_USE = _zz_256_[0];
  assign decode_RS1_USE = _zz_257_[0];
  always @ (*) begin
    _zz_25_ = execute_REGFILE_WRITE_DATA;
    if(_zz_188_)begin
      _zz_25_ = execute_CsrPlugin_readData;
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
    if(_zz_132_)begin
      if((_zz_133_ == decode_INSTRUCTION[24 : 20]))begin
        decode_RS2 = _zz_134_;
      end
    end
    if(_zz_189_)begin
      if(_zz_190_)begin
        if(_zz_136_)begin
          decode_RS2 = _zz_46_;
        end
      end
    end
    if(_zz_191_)begin
      if(memory_BYPASSABLE_MEMORY_STAGE)begin
        if(_zz_138_)begin
          decode_RS2 = _zz_26_;
        end
      end
    end
    if(_zz_192_)begin
      if(execute_BYPASSABLE_EXECUTE_STAGE)begin
        if(_zz_140_)begin
          decode_RS2 = _zz_25_;
        end
      end
    end
  end

  always @ (*) begin
    decode_RS1 = decode_RegFilePlugin_rs1Data;
    if(_zz_132_)begin
      if((_zz_133_ == decode_INSTRUCTION[19 : 15]))begin
        decode_RS1 = _zz_134_;
      end
    end
    if(_zz_189_)begin
      if(_zz_190_)begin
        if(_zz_135_)begin
          decode_RS1 = _zz_46_;
        end
      end
    end
    if(_zz_191_)begin
      if(memory_BYPASSABLE_MEMORY_STAGE)begin
        if(_zz_137_)begin
          decode_RS1 = _zz_26_;
        end
      end
    end
    if(_zz_192_)begin
      if(execute_BYPASSABLE_EXECUTE_STAGE)begin
        if(_zz_139_)begin
          decode_RS1 = _zz_25_;
        end
      end
    end
  end

  assign memory_SHIFT_RIGHT = execute_to_memory_SHIFT_RIGHT;
  always @ (*) begin
    _zz_26_ = memory_REGFILE_WRITE_DATA;
    if(memory_arbitration_isValid)begin
      case(memory_SHIFT_CTRL)
        `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
          _zz_26_ = _zz_129_;
        end
        `ShiftCtrlEnum_defaultEncoding_SRL_1, `ShiftCtrlEnum_defaultEncoding_SRA_1 : begin
          _zz_26_ = memory_SHIFT_RIGHT;
        end
        default : begin
        end
      endcase
    end
    if(_zz_193_)begin
      _zz_26_ = memory_MulDivIterativePlugin_div_result;
    end
  end

  assign memory_SHIFT_CTRL = _zz_27_;
  assign execute_SHIFT_CTRL = _zz_28_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC2_FORCE_ZERO = decode_to_execute_SRC2_FORCE_ZERO;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_29_ = decode_PC;
  assign _zz_30_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_31_;
  assign _zz_32_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_33_;
  assign decode_SRC_USE_SUB_LESS = _zz_258_[0];
  assign decode_SRC_ADD_ZERO = _zz_259_[0];
  assign execute_SRC_ADD_SUB = execute_SrcPlugin_addSub;
  assign execute_SRC_LESS = execute_SrcPlugin_less;
  assign execute_ALU_CTRL = _zz_34_;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_ALU_BITWISE_CTRL = _zz_35_;
  assign _zz_36_ = writeBack_INSTRUCTION;
  assign _zz_37_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_38_ = 1'b0;
    if(lastStageRegFileWrite_valid)begin
      _zz_38_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_decompressor_output_payload_rsp_inst);
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_260_[0];
    if((decode_INSTRUCTION[11 : 7] == 5'h0))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  assign decode_LEGAL_INSTRUCTION = ({((decode_INSTRUCTION & 32'h0000005f) == 32'h00000017),{((decode_INSTRUCTION & 32'h0000007f) == 32'h0000006f),{((decode_INSTRUCTION & 32'h0000106f) == 32'h00000003),{((decode_INSTRUCTION & _zz_360_) == 32'h00001073),{(_zz_361_ == _zz_362_),{_zz_363_,{_zz_364_,_zz_365_}}}}}}} != 20'h0);
  assign writeBack_MEMORY_STORE = memory_to_writeBack_MEMORY_STORE;
  always @ (*) begin
    _zz_46_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_46_ = writeBack_DBusSimplePlugin_rspFormated;
    end
    if((writeBack_arbitration_isValid && writeBack_IS_MUL))begin
      case(_zz_230_)
        2'b00 : begin
          _zz_46_ = _zz_296_;
        end
        default : begin
          _zz_46_ = _zz_297_;
        end
      endcase
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_MEMORY_STORE = execute_to_memory_MEMORY_STORE;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_SRC_ADD = execute_SrcPlugin_addSub;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_MEMORY_STORE = decode_to_execute_MEMORY_STORE;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  assign execute_ALIGNEMENT_FAULT = 1'b0;
  assign decode_BRANCH_CTRL = _zz_47_;
  always @ (*) begin
    _zz_48_ = memory_FORMAL_PC_NEXT;
    if(BranchPlugin_jumpInterface_valid)begin
      _zz_48_ = BranchPlugin_jumpInterface_payload;
    end
  end

  always @ (*) begin
    _zz_49_ = execute_FORMAL_PC_NEXT;
    if(CsrPlugin_redoInterface_valid)begin
      _zz_49_ = CsrPlugin_redoInterface_payload;
    end
  end

  always @ (*) begin
    _zz_50_ = decode_FORMAL_PC_NEXT;
    if(IBusSimplePlugin_predictionJumpInterface_valid)begin
      _zz_50_ = IBusSimplePlugin_predictionJumpInterface_payload;
    end
  end

  assign decode_PC = IBusSimplePlugin_decodePc_pcReg;
  assign decode_INSTRUCTION = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  assign decode_IS_RVC = IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  assign decode_arbitration_haltItself = 1'b0;
  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((decode_arbitration_isValid && (_zz_130_ || _zz_131_)))begin
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
    if(_zz_194_)begin
      decode_arbitration_removeIt = 1'b1;
    end
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_flushIt = 1'b0;
  always @ (*) begin
    decode_arbitration_flushNext = 1'b0;
    if(IBusSimplePlugin_predictionJumpInterface_valid)begin
      decode_arbitration_flushNext = 1'b1;
    end
    if(_zz_194_)begin
      decode_arbitration_flushNext = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_100_)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if(_zz_195_)begin
      if((! execute_CsrPlugin_wfiWake))begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
    if(_zz_188_)begin
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
  end

  assign execute_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_removeIt = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  assign execute_arbitration_flushIt = 1'b0;
  always @ (*) begin
    execute_arbitration_flushNext = 1'b0;
    if(CsrPlugin_selfException_valid)begin
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
    if(_zz_193_)begin
      if(((! memory_MulDivIterativePlugin_frontendOk) || (! memory_MulDivIterativePlugin_div_done)))begin
        memory_arbitration_haltItself = 1'b1;
      end
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  assign memory_arbitration_flushIt = 1'b0;
  always @ (*) begin
    memory_arbitration_flushNext = 1'b0;
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
    if(_zz_196_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
    if(_zz_197_)begin
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
    if(_zz_196_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_197_)begin
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
    if(_zz_195_)begin
      CsrPlugin_inWfi = 1'b1;
    end
  end

  assign CsrPlugin_thirdPartyWake = 1'b0;
  always @ (*) begin
    CsrPlugin_jumpInterface_valid = 1'b0;
    if(_zz_196_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
    if(_zz_197_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_payload = 32'h0;
    if(_zz_196_)begin
      CsrPlugin_jumpInterface_payload = {CsrPlugin_xtvec_base,(2'b00)};
    end
    if(_zz_197_)begin
      case(_zz_198_)
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

  assign CsrPlugin_forceMachineWire = 1'b0;
  assign CsrPlugin_allowInterrupts = 1'b1;
  assign CsrPlugin_allowException = 1'b1;
  assign IBusSimplePlugin_externalFlush = ({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,{execute_arbitration_flushNext,decode_arbitration_flushNext}}} != (4'b0000));
  assign IBusSimplePlugin_jump_pcLoad_valid = ({BranchPlugin_jumpInterface_valid,{CsrPlugin_redoInterface_valid,{CsrPlugin_jumpInterface_valid,IBusSimplePlugin_predictionJumpInterface_valid}}} != (4'b0000));
  assign _zz_51_ = {IBusSimplePlugin_predictionJumpInterface_valid,{CsrPlugin_redoInterface_valid,{BranchPlugin_jumpInterface_valid,CsrPlugin_jumpInterface_valid}}};
  assign _zz_52_ = (_zz_51_ & (~ _zz_261_));
  assign _zz_53_ = _zz_52_[3];
  assign _zz_54_ = (_zz_52_[1] || _zz_53_);
  assign _zz_55_ = (_zz_52_[2] || _zz_53_);
  assign IBusSimplePlugin_jump_pcLoad_payload = _zz_187_;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_correction = 1'b0;
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_correction = 1'b1;
    end
  end

  assign IBusSimplePlugin_fetchPc_corrected = (IBusSimplePlugin_fetchPc_correction || IBusSimplePlugin_fetchPc_correctionReg);
  always @ (*) begin
    IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b0;
    if(IBusSimplePlugin_iBusRsp_stages_1_input_ready)begin
      IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_263_);
    if(IBusSimplePlugin_fetchPc_inc)begin
      IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_flushed = 1'b0;
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_flushed = 1'b1;
    end
  end

  assign IBusSimplePlugin_fetchPc_output_valid = ((! IBusSimplePlugin_fetcherHalt) && IBusSimplePlugin_fetchPc_booted);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_pc;
  always @ (*) begin
    IBusSimplePlugin_decodePc_flushed = 1'b0;
    if(_zz_199_)begin
      IBusSimplePlugin_decodePc_flushed = 1'b1;
    end
  end

  assign IBusSimplePlugin_decodePc_pcPlus = (IBusSimplePlugin_decodePc_pcReg + _zz_265_);
  assign IBusSimplePlugin_decodePc_injectedDecode = 1'b0;
  assign IBusSimplePlugin_iBusRsp_redoFetch = 1'b0;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_0_input_valid && ((! IBusSimplePlugin_cmdFork_canEmit) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b1;
    end
  end

  assign _zz_56_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_56_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_56_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
  assign _zz_57_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_57_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_57_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_flush = (IBusSimplePlugin_externalFlush || IBusSimplePlugin_iBusRsp_redoFetch);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = _zz_58_;
  assign _zz_58_ = ((1'b0 && (! _zz_59_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_59_ = _zz_60_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_59_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = IBusSimplePlugin_fetchPc_pcReg;
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
  assign _zz_61_ = IBusSimplePlugin_decompressor_raw[15 : 0];
  always @ (*) begin
    IBusSimplePlugin_decompressor_decompressed = 32'h0;
    case(_zz_226_)
      5'b00000 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{{{(2'b00),_zz_61_[10 : 7]},_zz_61_[12 : 11]},_zz_61_[5]},_zz_61_[6]},(2'b00)},5'h02},(3'b000)},_zz_63_},7'h13};
      end
      5'b00010 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{_zz_64_,_zz_62_},(3'b010)},_zz_63_},7'h03};
      end
      5'b00110 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_64_[11 : 5],_zz_63_},_zz_62_},(3'b010)},_zz_64_[4 : 0]},7'h23};
      end
      5'b01000 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{_zz_66_,_zz_61_[11 : 7]},(3'b000)},_zz_61_[11 : 7]},7'h13};
      end
      5'b01001 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_69_[20],_zz_69_[10 : 1]},_zz_69_[11]},_zz_69_[19 : 12]},_zz_81_},7'h6f};
      end
      5'b01010 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{_zz_66_,5'h0},(3'b000)},_zz_61_[11 : 7]},7'h13};
      end
      5'b01011 : begin
        IBusSimplePlugin_decompressor_decompressed = ((_zz_61_[11 : 7] == 5'h02) ? {{{{{{{{{_zz_73_,_zz_61_[4 : 3]},_zz_61_[5]},_zz_61_[2]},_zz_61_[6]},(4'b0000)},_zz_61_[11 : 7]},(3'b000)},_zz_61_[11 : 7]},7'h13} : {{_zz_266_[31 : 12],_zz_61_[11 : 7]},7'h37});
      end
      5'b01100 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{((_zz_61_[11 : 10] == (2'b10)) ? _zz_87_ : {{(1'b0),(_zz_378_ || _zz_379_)},5'h0}),(((! _zz_61_[11]) || _zz_83_) ? _zz_61_[6 : 2] : _zz_63_)},_zz_62_},_zz_85_},_zz_62_},(_zz_83_ ? 7'h13 : 7'h33)};
      end
      5'b01101 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_76_[20],_zz_76_[10 : 1]},_zz_76_[11]},_zz_76_[19 : 12]},_zz_80_},7'h6f};
      end
      5'b01110 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{_zz_79_[12],_zz_79_[10 : 5]},_zz_80_},_zz_62_},(3'b000)},_zz_79_[4 : 1]},_zz_79_[11]},7'h63};
      end
      5'b01111 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{_zz_79_[12],_zz_79_[10 : 5]},_zz_80_},_zz_62_},(3'b001)},_zz_79_[4 : 1]},_zz_79_[11]},7'h63};
      end
      5'b10000 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{7'h0,_zz_61_[6 : 2]},_zz_61_[11 : 7]},(3'b001)},_zz_61_[11 : 7]},7'h13};
      end
      5'b10010 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{{{{(4'b0000),_zz_61_[3 : 2]},_zz_61_[12]},_zz_61_[6 : 4]},(2'b00)},_zz_82_},(3'b010)},_zz_61_[11 : 7]},7'h03};
      end
      5'b10100 : begin
        IBusSimplePlugin_decompressor_decompressed = ((_zz_61_[12 : 2] == 11'h400) ? 32'h00100073 : ((_zz_61_[6 : 2] == 5'h0) ? {{{{12'h0,_zz_61_[11 : 7]},(3'b000)},(_zz_61_[12] ? _zz_81_ : _zz_80_)},7'h67} : {{{{{_zz_380_,_zz_381_},(_zz_382_ ? _zz_383_ : _zz_80_)},(3'b000)},_zz_61_[11 : 7]},7'h33}));
      end
      5'b10110 : begin
        IBusSimplePlugin_decompressor_decompressed = {{{{{_zz_267_[11 : 5],_zz_61_[6 : 2]},_zz_82_},(3'b010)},_zz_268_[4 : 0]},7'h23};
      end
      default : begin
      end
    endcase
  end

  assign _zz_62_ = {(2'b01),_zz_61_[9 : 7]};
  assign _zz_63_ = {(2'b01),_zz_61_[4 : 2]};
  assign _zz_64_ = {{{{5'h0,_zz_61_[5]},_zz_61_[12 : 10]},_zz_61_[6]},(2'b00)};
  assign _zz_65_ = _zz_61_[12];
  always @ (*) begin
    _zz_66_[11] = _zz_65_;
    _zz_66_[10] = _zz_65_;
    _zz_66_[9] = _zz_65_;
    _zz_66_[8] = _zz_65_;
    _zz_66_[7] = _zz_65_;
    _zz_66_[6] = _zz_65_;
    _zz_66_[5] = _zz_65_;
    _zz_66_[4 : 0] = _zz_61_[6 : 2];
  end

  assign _zz_67_ = _zz_61_[12];
  always @ (*) begin
    _zz_68_[9] = _zz_67_;
    _zz_68_[8] = _zz_67_;
    _zz_68_[7] = _zz_67_;
    _zz_68_[6] = _zz_67_;
    _zz_68_[5] = _zz_67_;
    _zz_68_[4] = _zz_67_;
    _zz_68_[3] = _zz_67_;
    _zz_68_[2] = _zz_67_;
    _zz_68_[1] = _zz_67_;
    _zz_68_[0] = _zz_67_;
  end

  assign _zz_69_ = {{{{{{{{_zz_68_,_zz_61_[8]},_zz_61_[10 : 9]},_zz_61_[6]},_zz_61_[7]},_zz_61_[2]},_zz_61_[11]},_zz_61_[5 : 3]},(1'b0)};
  assign _zz_70_ = _zz_61_[12];
  always @ (*) begin
    _zz_71_[14] = _zz_70_;
    _zz_71_[13] = _zz_70_;
    _zz_71_[12] = _zz_70_;
    _zz_71_[11] = _zz_70_;
    _zz_71_[10] = _zz_70_;
    _zz_71_[9] = _zz_70_;
    _zz_71_[8] = _zz_70_;
    _zz_71_[7] = _zz_70_;
    _zz_71_[6] = _zz_70_;
    _zz_71_[5] = _zz_70_;
    _zz_71_[4] = _zz_70_;
    _zz_71_[3] = _zz_70_;
    _zz_71_[2] = _zz_70_;
    _zz_71_[1] = _zz_70_;
    _zz_71_[0] = _zz_70_;
  end

  assign _zz_72_ = _zz_61_[12];
  always @ (*) begin
    _zz_73_[2] = _zz_72_;
    _zz_73_[1] = _zz_72_;
    _zz_73_[0] = _zz_72_;
  end

  assign _zz_74_ = _zz_61_[12];
  always @ (*) begin
    _zz_75_[9] = _zz_74_;
    _zz_75_[8] = _zz_74_;
    _zz_75_[7] = _zz_74_;
    _zz_75_[6] = _zz_74_;
    _zz_75_[5] = _zz_74_;
    _zz_75_[4] = _zz_74_;
    _zz_75_[3] = _zz_74_;
    _zz_75_[2] = _zz_74_;
    _zz_75_[1] = _zz_74_;
    _zz_75_[0] = _zz_74_;
  end

  assign _zz_76_ = {{{{{{{{_zz_75_,_zz_61_[8]},_zz_61_[10 : 9]},_zz_61_[6]},_zz_61_[7]},_zz_61_[2]},_zz_61_[11]},_zz_61_[5 : 3]},(1'b0)};
  assign _zz_77_ = _zz_61_[12];
  always @ (*) begin
    _zz_78_[4] = _zz_77_;
    _zz_78_[3] = _zz_77_;
    _zz_78_[2] = _zz_77_;
    _zz_78_[1] = _zz_77_;
    _zz_78_[0] = _zz_77_;
  end

  assign _zz_79_ = {{{{{_zz_78_,_zz_61_[6 : 5]},_zz_61_[2]},_zz_61_[11 : 10]},_zz_61_[4 : 3]},(1'b0)};
  assign _zz_80_ = 5'h0;
  assign _zz_81_ = 5'h01;
  assign _zz_82_ = 5'h02;
  assign _zz_83_ = (_zz_61_[11 : 10] != (2'b11));
  always @ (*) begin
    case(_zz_227_)
      2'b00 : begin
        _zz_84_ = (3'b000);
      end
      2'b01 : begin
        _zz_84_ = (3'b100);
      end
      2'b10 : begin
        _zz_84_ = (3'b110);
      end
      default : begin
        _zz_84_ = (3'b111);
      end
    endcase
  end

  always @ (*) begin
    case(_zz_228_)
      2'b00 : begin
        _zz_85_ = (3'b101);
      end
      2'b01 : begin
        _zz_85_ = (3'b101);
      end
      2'b10 : begin
        _zz_85_ = (3'b111);
      end
      default : begin
        _zz_85_ = _zz_84_;
      end
    endcase
  end

  assign _zz_86_ = _zz_61_[12];
  always @ (*) begin
    _zz_87_[6] = _zz_86_;
    _zz_87_[5] = _zz_86_;
    _zz_87_[4] = _zz_86_;
    _zz_87_[3] = _zz_86_;
    _zz_87_[2] = _zz_86_;
    _zz_87_[1] = _zz_86_;
    _zz_87_[0] = _zz_86_;
  end

  assign IBusSimplePlugin_decompressor_output_valid = (IBusSimplePlugin_decompressor_input_valid && (! ((IBusSimplePlugin_decompressor_throw2Bytes && (! IBusSimplePlugin_decompressor_bufferValid)) && (! IBusSimplePlugin_decompressor_isInputHighRvc))));
  assign IBusSimplePlugin_decompressor_output_payload_pc = IBusSimplePlugin_decompressor_input_payload_pc;
  assign IBusSimplePlugin_decompressor_output_payload_isRvc = IBusSimplePlugin_decompressor_isRvc;
  assign IBusSimplePlugin_decompressor_output_payload_rsp_inst = (IBusSimplePlugin_decompressor_isRvc ? IBusSimplePlugin_decompressor_decompressed : IBusSimplePlugin_decompressor_raw);
  assign IBusSimplePlugin_decompressor_input_ready = (IBusSimplePlugin_decompressor_output_ready && (((! IBusSimplePlugin_iBusRsp_stages_1_input_valid) || IBusSimplePlugin_decompressor_flushNext) || ((! (IBusSimplePlugin_decompressor_bufferValid && IBusSimplePlugin_decompressor_isInputHighRvc)) && (! (((! IBusSimplePlugin_decompressor_unaligned) && IBusSimplePlugin_decompressor_isInputLowRvc) && IBusSimplePlugin_decompressor_isInputHighRvc)))));
  assign IBusSimplePlugin_decompressor_bufferFill = (((((! IBusSimplePlugin_decompressor_unaligned) && IBusSimplePlugin_decompressor_isInputLowRvc) && (! IBusSimplePlugin_decompressor_isInputHighRvc)) || (IBusSimplePlugin_decompressor_bufferValid && (! IBusSimplePlugin_decompressor_isInputHighRvc))) || ((IBusSimplePlugin_decompressor_throw2Bytes && (! IBusSimplePlugin_decompressor_isRvc)) && (! IBusSimplePlugin_decompressor_isInputHighRvc)));
  assign IBusSimplePlugin_decompressor_output_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_88_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_89_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_90_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_91_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_92_;
  assign IBusSimplePlugin_pcValids_0 = IBusSimplePlugin_injector_nextPcCalc_valids_0;
  assign IBusSimplePlugin_pcValids_1 = IBusSimplePlugin_injector_nextPcCalc_valids_1;
  assign IBusSimplePlugin_pcValids_2 = IBusSimplePlugin_injector_nextPcCalc_valids_2;
  assign IBusSimplePlugin_pcValids_3 = IBusSimplePlugin_injector_nextPcCalc_valids_3;
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  assign decode_arbitration_isValid = IBusSimplePlugin_injector_decodeInput_valid;
  assign _zz_93_ = _zz_269_[11];
  always @ (*) begin
    _zz_94_[18] = _zz_93_;
    _zz_94_[17] = _zz_93_;
    _zz_94_[16] = _zz_93_;
    _zz_94_[15] = _zz_93_;
    _zz_94_[14] = _zz_93_;
    _zz_94_[13] = _zz_93_;
    _zz_94_[12] = _zz_93_;
    _zz_94_[11] = _zz_93_;
    _zz_94_[10] = _zz_93_;
    _zz_94_[9] = _zz_93_;
    _zz_94_[8] = _zz_93_;
    _zz_94_[7] = _zz_93_;
    _zz_94_[6] = _zz_93_;
    _zz_94_[5] = _zz_93_;
    _zz_94_[4] = _zz_93_;
    _zz_94_[3] = _zz_93_;
    _zz_94_[2] = _zz_93_;
    _zz_94_[1] = _zz_93_;
    _zz_94_[0] = _zz_93_;
  end

  assign IBusSimplePlugin_decodePrediction_cmd_hadBranch = ((decode_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JAL) || ((decode_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_B) && _zz_270_[31]));
  assign IBusSimplePlugin_predictionJumpInterface_valid = (decode_arbitration_isValid && IBusSimplePlugin_decodePrediction_cmd_hadBranch);
  assign _zz_95_ = _zz_271_[19];
  always @ (*) begin
    _zz_96_[10] = _zz_95_;
    _zz_96_[9] = _zz_95_;
    _zz_96_[8] = _zz_95_;
    _zz_96_[7] = _zz_95_;
    _zz_96_[6] = _zz_95_;
    _zz_96_[5] = _zz_95_;
    _zz_96_[4] = _zz_95_;
    _zz_96_[3] = _zz_95_;
    _zz_96_[2] = _zz_95_;
    _zz_96_[1] = _zz_95_;
    _zz_96_[0] = _zz_95_;
  end

  assign _zz_97_ = _zz_272_[11];
  always @ (*) begin
    _zz_98_[18] = _zz_97_;
    _zz_98_[17] = _zz_97_;
    _zz_98_[16] = _zz_97_;
    _zz_98_[15] = _zz_97_;
    _zz_98_[14] = _zz_97_;
    _zz_98_[13] = _zz_97_;
    _zz_98_[12] = _zz_97_;
    _zz_98_[11] = _zz_97_;
    _zz_98_[10] = _zz_97_;
    _zz_98_[9] = _zz_97_;
    _zz_98_[8] = _zz_97_;
    _zz_98_[7] = _zz_97_;
    _zz_98_[6] = _zz_97_;
    _zz_98_[5] = _zz_97_;
    _zz_98_[4] = _zz_97_;
    _zz_98_[3] = _zz_97_;
    _zz_98_[2] = _zz_97_;
    _zz_98_[1] = _zz_97_;
    _zz_98_[0] = _zz_97_;
  end

  assign IBusSimplePlugin_predictionJumpInterface_payload = (decode_PC + ((decode_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JAL) ? {{_zz_96_,{{{_zz_384_,_zz_385_},_zz_386_},decode_INSTRUCTION[30 : 21]}},1'b0} : {{_zz_98_,{{{_zz_387_,_zz_388_},decode_INSTRUCTION[30 : 25]},decode_INSTRUCTION[11 : 8]}},1'b0}));
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pending_next = (_zz_273_ - _zz_277_);
  assign IBusSimplePlugin_cmdFork_canEmit = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && (IBusSimplePlugin_pending_value != (3'b111)));
  assign IBusSimplePlugin_cmd_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && IBusSimplePlugin_cmdFork_canEmit);
  assign IBusSimplePlugin_pending_inc = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_iBusRsp_stages_0_input_payload[31 : 2],(2'b00)};
  assign IBusSimplePlugin_rspJoin_rspBuffer_flush = ((IBusSimplePlugin_rspJoin_rspBuffer_discardCounter != (3'b000)) || IBusSimplePlugin_iBusRsp_flush);
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_valid = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter == (3'b000)));
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  assign _zz_183_ = (IBusSimplePlugin_rspJoin_rspBuffer_output_ready || IBusSimplePlugin_rspJoin_rspBuffer_flush);
  assign IBusSimplePlugin_pending_dec = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && _zz_183_);
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
    if(_zz_200_)begin
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
  assign _zz_99_ = (! IBusSimplePlugin_rspJoin_exceptionDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_output_ready && _zz_99_);
  assign IBusSimplePlugin_iBusRsp_output_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_99_);
  assign IBusSimplePlugin_iBusRsp_output_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_output_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_output_payload_rsp_inst = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_output_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  always @ (*) begin
    IBusSimplePlugin_decodeExceptionPort_payload_code = (4'bxxxx);
    if(_zz_200_)begin
      IBusSimplePlugin_decodeExceptionPort_payload_code = (4'b0001);
    end
  end

  assign IBusSimplePlugin_decodeExceptionPort_payload_badAddr = {IBusSimplePlugin_rspJoin_join_payload_pc[31 : 2],(2'b00)};
  assign IBusSimplePlugin_decodeExceptionPort_valid = (IBusSimplePlugin_rspJoin_exceptionDetected && IBusSimplePlugin_iBusRsp_readyForError);
  assign _zz_100_ = 1'b0;
  always @ (*) begin
    execute_DBusSimplePlugin_skipCmd = 1'b0;
    if(execute_ALIGNEMENT_FAULT)begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
  end

  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_100_));
  assign dBus_cmd_payload_wr = execute_MEMORY_STORE;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_101_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_101_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_101_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_101_;
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_102_ = (4'b0001);
      end
      2'b01 : begin
        _zz_102_ = (4'b0011);
      end
      default : begin
        _zz_102_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_102_ <<< dBus_cmd_payload_address[1 : 0]);
  assign dBus_cmd_payload_address = execute_SRC_ADD;
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

  assign _zz_103_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_104_[31] = _zz_103_;
    _zz_104_[30] = _zz_103_;
    _zz_104_[29] = _zz_103_;
    _zz_104_[28] = _zz_103_;
    _zz_104_[27] = _zz_103_;
    _zz_104_[26] = _zz_103_;
    _zz_104_[25] = _zz_103_;
    _zz_104_[24] = _zz_103_;
    _zz_104_[23] = _zz_103_;
    _zz_104_[22] = _zz_103_;
    _zz_104_[21] = _zz_103_;
    _zz_104_[20] = _zz_103_;
    _zz_104_[19] = _zz_103_;
    _zz_104_[18] = _zz_103_;
    _zz_104_[17] = _zz_103_;
    _zz_104_[16] = _zz_103_;
    _zz_104_[15] = _zz_103_;
    _zz_104_[14] = _zz_103_;
    _zz_104_[13] = _zz_103_;
    _zz_104_[12] = _zz_103_;
    _zz_104_[11] = _zz_103_;
    _zz_104_[10] = _zz_103_;
    _zz_104_[9] = _zz_103_;
    _zz_104_[8] = _zz_103_;
    _zz_104_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_105_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_106_[31] = _zz_105_;
    _zz_106_[30] = _zz_105_;
    _zz_106_[29] = _zz_105_;
    _zz_106_[28] = _zz_105_;
    _zz_106_[27] = _zz_105_;
    _zz_106_[26] = _zz_105_;
    _zz_106_[25] = _zz_105_;
    _zz_106_[24] = _zz_105_;
    _zz_106_[23] = _zz_105_;
    _zz_106_[22] = _zz_105_;
    _zz_106_[21] = _zz_105_;
    _zz_106_[20] = _zz_105_;
    _zz_106_[19] = _zz_105_;
    _zz_106_[18] = _zz_105_;
    _zz_106_[17] = _zz_105_;
    _zz_106_[16] = _zz_105_;
    _zz_106_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_229_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_104_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_106_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign _zz_108_ = ((decode_INSTRUCTION & 32'h00001000) == 32'h0);
  assign _zz_109_ = ((decode_INSTRUCTION & 32'h00000004) == 32'h00000004);
  assign _zz_110_ = ((decode_INSTRUCTION & 32'h00000048) == 32'h00000048);
  assign _zz_111_ = ((decode_INSTRUCTION & 32'h00006004) == 32'h00002000);
  assign _zz_112_ = ((decode_INSTRUCTION & 32'h00004050) == 32'h00004050);
  assign _zz_107_ = {({(_zz_389_ == _zz_390_),(_zz_391_ == _zz_392_)} != (2'b00)),{({_zz_393_,{_zz_394_,_zz_395_}} != (3'b000)),{({_zz_396_,_zz_397_} != (2'b00)),{(_zz_398_ != _zz_399_),{_zz_400_,{_zz_401_,_zz_402_}}}}}};
  assign _zz_113_ = _zz_107_[8 : 7];
  assign _zz_45_ = _zz_113_;
  assign _zz_114_ = _zz_107_[11 : 10];
  assign _zz_44_ = _zz_114_;
  assign _zz_115_ = _zz_107_[14 : 13];
  assign _zz_43_ = _zz_115_;
  assign _zz_116_ = _zz_107_[17 : 16];
  assign _zz_42_ = _zz_116_;
  assign _zz_117_ = _zz_107_[25 : 24];
  assign _zz_41_ = _zz_117_;
  assign _zz_118_ = _zz_107_[27 : 26];
  assign _zz_40_ = _zz_118_;
  assign _zz_119_ = _zz_107_[29 : 28];
  assign _zz_39_ = _zz_119_;
  assign decodeExceptionPort_valid = (decode_arbitration_isValid && (! decode_LEGAL_INSTRUCTION));
  assign decodeExceptionPort_payload_code = (4'b0010);
  assign decodeExceptionPort_payload_badAddr = decode_INSTRUCTION;
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_185_;
  assign decode_RegFilePlugin_rs2Data = _zz_186_;
  always @ (*) begin
    lastStageRegFileWrite_valid = (_zz_37_ && writeBack_arbitration_isFiring);
    if(_zz_120_)begin
      lastStageRegFileWrite_valid = 1'b1;
    end
  end

  assign lastStageRegFileWrite_payload_address = _zz_36_[11 : 7];
  assign lastStageRegFileWrite_payload_data = _zz_46_;
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
        _zz_121_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_121_ = {31'd0, _zz_282_};
      end
      default : begin
        _zz_121_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_122_ = _zz_32_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_122_ = {29'd0, _zz_283_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_122_ = {decode_INSTRUCTION[31 : 12],12'h0};
      end
      default : begin
        _zz_122_ = {27'd0, _zz_284_};
      end
    endcase
  end

  assign _zz_123_ = _zz_285_[11];
  always @ (*) begin
    _zz_124_[19] = _zz_123_;
    _zz_124_[18] = _zz_123_;
    _zz_124_[17] = _zz_123_;
    _zz_124_[16] = _zz_123_;
    _zz_124_[15] = _zz_123_;
    _zz_124_[14] = _zz_123_;
    _zz_124_[13] = _zz_123_;
    _zz_124_[12] = _zz_123_;
    _zz_124_[11] = _zz_123_;
    _zz_124_[10] = _zz_123_;
    _zz_124_[9] = _zz_123_;
    _zz_124_[8] = _zz_123_;
    _zz_124_[7] = _zz_123_;
    _zz_124_[6] = _zz_123_;
    _zz_124_[5] = _zz_123_;
    _zz_124_[4] = _zz_123_;
    _zz_124_[3] = _zz_123_;
    _zz_124_[2] = _zz_123_;
    _zz_124_[1] = _zz_123_;
    _zz_124_[0] = _zz_123_;
  end

  assign _zz_125_ = _zz_286_[11];
  always @ (*) begin
    _zz_126_[19] = _zz_125_;
    _zz_126_[18] = _zz_125_;
    _zz_126_[17] = _zz_125_;
    _zz_126_[16] = _zz_125_;
    _zz_126_[15] = _zz_125_;
    _zz_126_[14] = _zz_125_;
    _zz_126_[13] = _zz_125_;
    _zz_126_[12] = _zz_125_;
    _zz_126_[11] = _zz_125_;
    _zz_126_[10] = _zz_125_;
    _zz_126_[9] = _zz_125_;
    _zz_126_[8] = _zz_125_;
    _zz_126_[7] = _zz_125_;
    _zz_126_[6] = _zz_125_;
    _zz_126_[5] = _zz_125_;
    _zz_126_[4] = _zz_125_;
    _zz_126_[3] = _zz_125_;
    _zz_126_[2] = _zz_125_;
    _zz_126_[1] = _zz_125_;
    _zz_126_[0] = _zz_125_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_127_ = _zz_30_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_127_ = {_zz_124_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_127_ = {_zz_126_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_127_ = _zz_29_;
      end
    endcase
  end

  always @ (*) begin
    execute_SrcPlugin_addSub = _zz_287_;
    if(execute_SRC2_FORCE_ZERO)begin
      execute_SrcPlugin_addSub = execute_SRC1;
    end
  end

  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign execute_FullBarrelShifterPlugin_amplitude = execute_SRC2[4 : 0];
  always @ (*) begin
    _zz_128_[0] = execute_SRC1[31];
    _zz_128_[1] = execute_SRC1[30];
    _zz_128_[2] = execute_SRC1[29];
    _zz_128_[3] = execute_SRC1[28];
    _zz_128_[4] = execute_SRC1[27];
    _zz_128_[5] = execute_SRC1[26];
    _zz_128_[6] = execute_SRC1[25];
    _zz_128_[7] = execute_SRC1[24];
    _zz_128_[8] = execute_SRC1[23];
    _zz_128_[9] = execute_SRC1[22];
    _zz_128_[10] = execute_SRC1[21];
    _zz_128_[11] = execute_SRC1[20];
    _zz_128_[12] = execute_SRC1[19];
    _zz_128_[13] = execute_SRC1[18];
    _zz_128_[14] = execute_SRC1[17];
    _zz_128_[15] = execute_SRC1[16];
    _zz_128_[16] = execute_SRC1[15];
    _zz_128_[17] = execute_SRC1[14];
    _zz_128_[18] = execute_SRC1[13];
    _zz_128_[19] = execute_SRC1[12];
    _zz_128_[20] = execute_SRC1[11];
    _zz_128_[21] = execute_SRC1[10];
    _zz_128_[22] = execute_SRC1[9];
    _zz_128_[23] = execute_SRC1[8];
    _zz_128_[24] = execute_SRC1[7];
    _zz_128_[25] = execute_SRC1[6];
    _zz_128_[26] = execute_SRC1[5];
    _zz_128_[27] = execute_SRC1[4];
    _zz_128_[28] = execute_SRC1[3];
    _zz_128_[29] = execute_SRC1[2];
    _zz_128_[30] = execute_SRC1[1];
    _zz_128_[31] = execute_SRC1[0];
  end

  assign execute_FullBarrelShifterPlugin_reversed = ((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SLL_1) ? _zz_128_ : execute_SRC1);
  always @ (*) begin
    _zz_129_[0] = memory_SHIFT_RIGHT[31];
    _zz_129_[1] = memory_SHIFT_RIGHT[30];
    _zz_129_[2] = memory_SHIFT_RIGHT[29];
    _zz_129_[3] = memory_SHIFT_RIGHT[28];
    _zz_129_[4] = memory_SHIFT_RIGHT[27];
    _zz_129_[5] = memory_SHIFT_RIGHT[26];
    _zz_129_[6] = memory_SHIFT_RIGHT[25];
    _zz_129_[7] = memory_SHIFT_RIGHT[24];
    _zz_129_[8] = memory_SHIFT_RIGHT[23];
    _zz_129_[9] = memory_SHIFT_RIGHT[22];
    _zz_129_[10] = memory_SHIFT_RIGHT[21];
    _zz_129_[11] = memory_SHIFT_RIGHT[20];
    _zz_129_[12] = memory_SHIFT_RIGHT[19];
    _zz_129_[13] = memory_SHIFT_RIGHT[18];
    _zz_129_[14] = memory_SHIFT_RIGHT[17];
    _zz_129_[15] = memory_SHIFT_RIGHT[16];
    _zz_129_[16] = memory_SHIFT_RIGHT[15];
    _zz_129_[17] = memory_SHIFT_RIGHT[14];
    _zz_129_[18] = memory_SHIFT_RIGHT[13];
    _zz_129_[19] = memory_SHIFT_RIGHT[12];
    _zz_129_[20] = memory_SHIFT_RIGHT[11];
    _zz_129_[21] = memory_SHIFT_RIGHT[10];
    _zz_129_[22] = memory_SHIFT_RIGHT[9];
    _zz_129_[23] = memory_SHIFT_RIGHT[8];
    _zz_129_[24] = memory_SHIFT_RIGHT[7];
    _zz_129_[25] = memory_SHIFT_RIGHT[6];
    _zz_129_[26] = memory_SHIFT_RIGHT[5];
    _zz_129_[27] = memory_SHIFT_RIGHT[4];
    _zz_129_[28] = memory_SHIFT_RIGHT[3];
    _zz_129_[29] = memory_SHIFT_RIGHT[2];
    _zz_129_[30] = memory_SHIFT_RIGHT[1];
    _zz_129_[31] = memory_SHIFT_RIGHT[0];
  end

  always @ (*) begin
    _zz_130_ = 1'b0;
    if(_zz_201_)begin
      if(_zz_202_)begin
        if(_zz_135_)begin
          _zz_130_ = 1'b1;
        end
      end
    end
    if(_zz_203_)begin
      if(_zz_204_)begin
        if(_zz_137_)begin
          _zz_130_ = 1'b1;
        end
      end
    end
    if(_zz_205_)begin
      if(_zz_206_)begin
        if(_zz_139_)begin
          _zz_130_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_130_ = 1'b0;
    end
  end

  always @ (*) begin
    _zz_131_ = 1'b0;
    if(_zz_201_)begin
      if(_zz_202_)begin
        if(_zz_136_)begin
          _zz_131_ = 1'b1;
        end
      end
    end
    if(_zz_203_)begin
      if(_zz_204_)begin
        if(_zz_138_)begin
          _zz_131_ = 1'b1;
        end
      end
    end
    if(_zz_205_)begin
      if(_zz_206_)begin
        if(_zz_140_)begin
          _zz_131_ = 1'b1;
        end
      end
    end
    if((! decode_RS2_USE))begin
      _zz_131_ = 1'b0;
    end
  end

  assign _zz_135_ = (writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_136_ = (writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign _zz_137_ = (memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_138_ = (memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign _zz_139_ = (execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]);
  assign _zz_140_ = (execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]);
  assign execute_MulPlugin_a = execute_RS1;
  assign execute_MulPlugin_b = execute_RS2;
  always @ (*) begin
    case(_zz_207_)
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
    case(_zz_207_)
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
  assign writeBack_MulPlugin_result = ($signed(_zz_294_) + $signed(_zz_295_));
  assign memory_MulDivIterativePlugin_frontendOk = 1'b1;
  always @ (*) begin
    memory_MulDivIterativePlugin_div_counter_willIncrement = 1'b0;
    if(_zz_193_)begin
      if(_zz_208_)begin
        memory_MulDivIterativePlugin_div_counter_willIncrement = 1'b1;
      end
    end
  end

  always @ (*) begin
    memory_MulDivIterativePlugin_div_counter_willClear = 1'b0;
    if(_zz_209_)begin
      memory_MulDivIterativePlugin_div_counter_willClear = 1'b1;
    end
  end

  assign memory_MulDivIterativePlugin_div_counter_willOverflowIfInc = (memory_MulDivIterativePlugin_div_counter_value == 6'h21);
  assign memory_MulDivIterativePlugin_div_counter_willOverflow = (memory_MulDivIterativePlugin_div_counter_willOverflowIfInc && memory_MulDivIterativePlugin_div_counter_willIncrement);
  always @ (*) begin
    if(memory_MulDivIterativePlugin_div_counter_willOverflow)begin
      memory_MulDivIterativePlugin_div_counter_valueNext = 6'h0;
    end else begin
      memory_MulDivIterativePlugin_div_counter_valueNext = (memory_MulDivIterativePlugin_div_counter_value + _zz_299_);
    end
    if(memory_MulDivIterativePlugin_div_counter_willClear)begin
      memory_MulDivIterativePlugin_div_counter_valueNext = 6'h0;
    end
  end

  assign _zz_141_ = memory_MulDivIterativePlugin_rs1[31 : 0];
  assign memory_MulDivIterativePlugin_div_stage_0_remainderShifted = {memory_MulDivIterativePlugin_accumulator[31 : 0],_zz_141_[31]};
  assign memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator = (memory_MulDivIterativePlugin_div_stage_0_remainderShifted - _zz_300_);
  assign memory_MulDivIterativePlugin_div_stage_0_outRemainder = ((! memory_MulDivIterativePlugin_div_stage_0_remainderMinusDenominator[32]) ? _zz_301_ : _zz_302_);
  assign memory_MulDivIterativePlugin_div_stage_0_outNumerator = _zz_303_[31:0];
  assign _zz_142_ = (memory_INSTRUCTION[13] ? memory_MulDivIterativePlugin_accumulator[31 : 0] : memory_MulDivIterativePlugin_rs1[31 : 0]);
  assign _zz_143_ = (execute_RS2[31] && execute_IS_RS2_SIGNED);
  assign _zz_144_ = (1'b0 || ((execute_IS_DIV && execute_RS1[31]) && execute_IS_RS1_SIGNED));
  always @ (*) begin
    _zz_145_[32] = (execute_IS_RS1_SIGNED && execute_RS1[31]);
    _zz_145_[31 : 0] = execute_RS1;
  end

  always @ (*) begin
    CsrPlugin_privilege = _zz_146_;
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
  assign _zz_147_ = (CsrPlugin_sip_STIP && CsrPlugin_sie_STIE);
  assign _zz_148_ = (CsrPlugin_sip_SSIP && CsrPlugin_sie_SSIE);
  assign _zz_149_ = (CsrPlugin_sip_SEIP_OR && CsrPlugin_sie_SEIE);
  assign _zz_150_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_151_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_152_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
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
  assign _zz_153_ = {decodeExceptionPort_valid,IBusSimplePlugin_decodeExceptionPort_valid};
  assign _zz_154_ = _zz_313_[0];
  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_decode = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
    if(_zz_194_)begin
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
      if(execute_CSR_WRITE_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if(_zz_210_)begin
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
    if(_zz_211_)begin
      CsrPlugin_selfException_valid = 1'b1;
    end
    if(_zz_212_)begin
      CsrPlugin_selfException_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_payload_code = (4'bxxxx);
    if(_zz_211_)begin
      CsrPlugin_selfException_payload_code = (4'b0010);
    end
    if(_zz_212_)begin
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
    if(_zz_210_)begin
      execute_CsrPlugin_writeInstruction = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
    if(_zz_210_)begin
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
    case(_zz_231_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readToWriteData & (~ execute_SRC1)) : (execute_CsrPlugin_readToWriteData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_155_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_155_ == (3'b000))) begin
        _zz_156_ = execute_BranchPlugin_eq;
    end else if((_zz_155_ == (3'b001))) begin
        _zz_156_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_155_ & (3'b101)) == (3'b101)))) begin
        _zz_156_ = (! execute_SRC_LESS);
    end else begin
        _zz_156_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_157_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_157_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_157_ = 1'b1;
      end
      default : begin
        _zz_157_ = _zz_156_;
      end
    endcase
  end

  assign execute_BranchPlugin_missAlignedTarget = 1'b0;
  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        execute_BranchPlugin_branch_src1 = execute_RS1;
      end
      default : begin
        execute_BranchPlugin_branch_src1 = execute_PC;
      end
    endcase
  end

  assign _zz_158_ = _zz_315_[11];
  always @ (*) begin
    _zz_159_[19] = _zz_158_;
    _zz_159_[18] = _zz_158_;
    _zz_159_[17] = _zz_158_;
    _zz_159_[16] = _zz_158_;
    _zz_159_[15] = _zz_158_;
    _zz_159_[14] = _zz_158_;
    _zz_159_[13] = _zz_158_;
    _zz_159_[12] = _zz_158_;
    _zz_159_[11] = _zz_158_;
    _zz_159_[10] = _zz_158_;
    _zz_159_[9] = _zz_158_;
    _zz_159_[8] = _zz_158_;
    _zz_159_[7] = _zz_158_;
    _zz_159_[6] = _zz_158_;
    _zz_159_[5] = _zz_158_;
    _zz_159_[4] = _zz_158_;
    _zz_159_[3] = _zz_158_;
    _zz_159_[2] = _zz_158_;
    _zz_159_[1] = _zz_158_;
    _zz_159_[0] = _zz_158_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        execute_BranchPlugin_branch_src2 = {_zz_159_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        execute_BranchPlugin_branch_src2 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JAL) ? {{_zz_161_,{{{_zz_526_,execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0} : {{_zz_163_,{{{_zz_527_,_zz_528_},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0});
        if(execute_PREDICTION_HAD_BRANCHED2)begin
          execute_BranchPlugin_branch_src2 = {29'd0, _zz_318_};
        end
      end
    endcase
  end

  assign _zz_160_ = _zz_316_[19];
  always @ (*) begin
    _zz_161_[10] = _zz_160_;
    _zz_161_[9] = _zz_160_;
    _zz_161_[8] = _zz_160_;
    _zz_161_[7] = _zz_160_;
    _zz_161_[6] = _zz_160_;
    _zz_161_[5] = _zz_160_;
    _zz_161_[4] = _zz_160_;
    _zz_161_[3] = _zz_160_;
    _zz_161_[2] = _zz_160_;
    _zz_161_[1] = _zz_160_;
    _zz_161_[0] = _zz_160_;
  end

  assign _zz_162_ = _zz_317_[11];
  always @ (*) begin
    _zz_163_[18] = _zz_162_;
    _zz_163_[17] = _zz_162_;
    _zz_163_[16] = _zz_162_;
    _zz_163_[15] = _zz_162_;
    _zz_163_[14] = _zz_162_;
    _zz_163_[13] = _zz_162_;
    _zz_163_[12] = _zz_162_;
    _zz_163_[11] = _zz_162_;
    _zz_163_[10] = _zz_162_;
    _zz_163_[9] = _zz_162_;
    _zz_163_[8] = _zz_162_;
    _zz_163_[7] = _zz_162_;
    _zz_163_[6] = _zz_162_;
    _zz_163_[5] = _zz_162_;
    _zz_163_[4] = _zz_162_;
    _zz_163_[3] = _zz_162_;
    _zz_163_[2] = _zz_162_;
    _zz_163_[1] = _zz_162_;
    _zz_163_[0] = _zz_162_;
  end

  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign BranchPlugin_jumpInterface_valid = ((memory_arbitration_isValid && memory_BRANCH_DO) && (! 1'b0));
  assign BranchPlugin_jumpInterface_payload = memory_BRANCH_CALC;
  assign IBusSimplePlugin_decodePrediction_rsp_wasWrong = BranchPlugin_jumpInterface_valid;
  assign _zz_20_ = decode_BRANCH_CTRL;
  assign _zz_47_ = _zz_45_;
  assign _zz_21_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_18_ = decode_ALU_CTRL;
  assign _zz_16_ = _zz_43_;
  assign _zz_34_ = decode_to_execute_ALU_CTRL;
  assign _zz_33_ = _zz_40_;
  assign _zz_31_ = _zz_42_;
  assign _zz_15_ = decode_SHIFT_CTRL;
  assign _zz_12_ = execute_SHIFT_CTRL;
  assign _zz_13_ = _zz_39_;
  assign _zz_28_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_27_ = execute_to_memory_SHIFT_CTRL;
  assign _zz_10_ = decode_ENV_CTRL;
  assign _zz_7_ = execute_ENV_CTRL;
  assign _zz_5_ = memory_ENV_CTRL;
  assign _zz_8_ = _zz_44_;
  assign _zz_23_ = decode_to_execute_ENV_CTRL;
  assign _zz_22_ = execute_to_memory_ENV_CTRL;
  assign _zz_24_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_3_ = decode_ALU_BITWISE_CTRL;
  assign _zz_1_ = _zz_41_;
  assign _zz_35_ = decode_to_execute_ALU_BITWISE_CTRL;
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
    _zz_164_ = 32'h0;
    if(execute_CsrPlugin_csr_3857)begin
      _zz_164_[0 : 0] = (1'b1);
    end
  end

  always @ (*) begin
    _zz_165_ = 32'h0;
    if(execute_CsrPlugin_csr_3858)begin
      _zz_165_[1 : 0] = (2'b10);
    end
  end

  always @ (*) begin
    _zz_166_ = 32'h0;
    if(execute_CsrPlugin_csr_3859)begin
      _zz_166_[1 : 0] = (2'b11);
    end
  end

  always @ (*) begin
    _zz_167_ = 32'h0;
    if(execute_CsrPlugin_csr_768)begin
      _zz_167_[12 : 11] = CsrPlugin_mstatus_MPP;
      _zz_167_[7 : 7] = CsrPlugin_mstatus_MPIE;
      _zz_167_[3 : 3] = CsrPlugin_mstatus_MIE;
      _zz_167_[8 : 8] = CsrPlugin_sstatus_SPP;
      _zz_167_[5 : 5] = CsrPlugin_sstatus_SPIE;
      _zz_167_[1 : 1] = CsrPlugin_sstatus_SIE;
    end
  end

  always @ (*) begin
    _zz_168_ = 32'h0;
    if(execute_CsrPlugin_csr_836)begin
      _zz_168_[11 : 11] = CsrPlugin_mip_MEIP;
      _zz_168_[7 : 7] = CsrPlugin_mip_MTIP;
      _zz_168_[3 : 3] = CsrPlugin_mip_MSIP;
      _zz_168_[5 : 5] = CsrPlugin_sip_STIP;
      _zz_168_[1 : 1] = CsrPlugin_sip_SSIP;
      _zz_168_[9 : 9] = CsrPlugin_sip_SEIP_OR;
    end
  end

  always @ (*) begin
    _zz_169_ = 32'h0;
    if(execute_CsrPlugin_csr_772)begin
      _zz_169_[11 : 11] = CsrPlugin_mie_MEIE;
      _zz_169_[7 : 7] = CsrPlugin_mie_MTIE;
      _zz_169_[3 : 3] = CsrPlugin_mie_MSIE;
      _zz_169_[9 : 9] = CsrPlugin_sie_SEIE;
      _zz_169_[5 : 5] = CsrPlugin_sie_STIE;
      _zz_169_[1 : 1] = CsrPlugin_sie_SSIE;
    end
  end

  always @ (*) begin
    _zz_170_ = 32'h0;
    if(execute_CsrPlugin_csr_833)begin
      _zz_170_[31 : 0] = CsrPlugin_mepc;
    end
  end

  always @ (*) begin
    _zz_171_ = 32'h0;
    if(execute_CsrPlugin_csr_832)begin
      _zz_171_[31 : 0] = CsrPlugin_mscratch;
    end
  end

  always @ (*) begin
    _zz_172_ = 32'h0;
    if(execute_CsrPlugin_csr_834)begin
      _zz_172_[31 : 31] = CsrPlugin_mcause_interrupt;
      _zz_172_[3 : 0] = CsrPlugin_mcause_exceptionCode;
    end
  end

  always @ (*) begin
    _zz_173_ = 32'h0;
    if(execute_CsrPlugin_csr_835)begin
      _zz_173_[31 : 0] = CsrPlugin_mtval;
    end
  end

  always @ (*) begin
    _zz_174_ = 32'h0;
    if(execute_CsrPlugin_csr_256)begin
      _zz_174_[8 : 8] = CsrPlugin_sstatus_SPP;
      _zz_174_[5 : 5] = CsrPlugin_sstatus_SPIE;
      _zz_174_[1 : 1] = CsrPlugin_sstatus_SIE;
    end
  end

  always @ (*) begin
    _zz_175_ = 32'h0;
    if(execute_CsrPlugin_csr_324)begin
      _zz_175_[5 : 5] = CsrPlugin_sip_STIP;
      _zz_175_[1 : 1] = CsrPlugin_sip_SSIP;
      _zz_175_[9 : 9] = CsrPlugin_sip_SEIP_OR;
    end
  end

  always @ (*) begin
    _zz_176_ = 32'h0;
    if(execute_CsrPlugin_csr_260)begin
      _zz_176_[9 : 9] = CsrPlugin_sie_SEIE;
      _zz_176_[5 : 5] = CsrPlugin_sie_STIE;
      _zz_176_[1 : 1] = CsrPlugin_sie_SSIE;
    end
  end

  always @ (*) begin
    _zz_177_ = 32'h0;
    if(execute_CsrPlugin_csr_261)begin
      _zz_177_[31 : 2] = CsrPlugin_stvec_base;
      _zz_177_[1 : 0] = CsrPlugin_stvec_mode;
    end
  end

  always @ (*) begin
    _zz_178_ = 32'h0;
    if(execute_CsrPlugin_csr_321)begin
      _zz_178_[31 : 0] = CsrPlugin_sepc;
    end
  end

  always @ (*) begin
    _zz_179_ = 32'h0;
    if(execute_CsrPlugin_csr_320)begin
      _zz_179_[31 : 0] = CsrPlugin_sscratch;
    end
  end

  always @ (*) begin
    _zz_180_ = 32'h0;
    if(execute_CsrPlugin_csr_322)begin
      _zz_180_[31 : 31] = CsrPlugin_scause_interrupt;
      _zz_180_[3 : 0] = CsrPlugin_scause_exceptionCode;
    end
  end

  always @ (*) begin
    _zz_181_ = 32'h0;
    if(execute_CsrPlugin_csr_323)begin
      _zz_181_[31 : 0] = CsrPlugin_stval;
    end
  end

  assign execute_CsrPlugin_readData = (((((_zz_164_ | _zz_165_) | (_zz_166_ | _zz_529_)) | ((_zz_167_ | _zz_168_) | (_zz_169_ | _zz_170_))) | (((_zz_171_ | _zz_172_) | (_zz_173_ | _zz_174_)) | ((_zz_175_ | _zz_176_) | (_zz_177_ | _zz_178_)))) | ((_zz_179_ | _zz_180_) | _zz_181_));
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
        _zz_182_ = (4'b0001);
      end
      2'b01 : begin
        _zz_182_ = (4'b0011);
      end
      default : begin
        _zz_182_ = (4'b1111);
      end
    endcase
  end

  always @ (*) begin
    dBusWishbone_SEL = (_zz_182_ <<< dBus_cmd_halfPipe_payload_address[1 : 0]);
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
  assign _zz_184_ = 1'b0;
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      IBusSimplePlugin_fetchPc_pcReg <= 32'h0;
      IBusSimplePlugin_fetchPc_correctionReg <= 1'b0;
      IBusSimplePlugin_fetchPc_booted <= 1'b0;
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      IBusSimplePlugin_decodePc_pcReg <= 32'h0;
      _zz_60_ <= 1'b0;
      IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
      IBusSimplePlugin_decompressor_throw2BytesReg <= 1'b0;
      _zz_88_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      IBusSimplePlugin_pending_value <= (3'b000);
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (3'b000);
      _zz_120_ <= 1'b1;
      _zz_132_ <= 1'b0;
      memory_MulDivIterativePlugin_div_counter_value <= 6'h0;
      _zz_146_ <= (2'b11);
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
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
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
      if(_zz_199_)begin
        IBusSimplePlugin_decodePc_pcReg <= IBusSimplePlugin_jump_pcLoad_payload;
      end
      if(IBusSimplePlugin_iBusRsp_flush)begin
        _zz_60_ <= 1'b0;
      end
      if(_zz_58_)begin
        _zz_60_ <= (IBusSimplePlugin_iBusRsp_stages_0_output_valid && (! 1'b0));
      end
      if((IBusSimplePlugin_decompressor_output_valid && IBusSimplePlugin_decompressor_output_ready))begin
        IBusSimplePlugin_decompressor_throw2BytesReg <= ((((! IBusSimplePlugin_decompressor_unaligned) && IBusSimplePlugin_decompressor_isInputLowRvc) && IBusSimplePlugin_decompressor_isInputHighRvc) || (IBusSimplePlugin_decompressor_bufferValid && IBusSimplePlugin_decompressor_isInputHighRvc));
      end
      if((IBusSimplePlugin_decompressor_output_ready && IBusSimplePlugin_decompressor_input_valid))begin
        IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
      end
      if(_zz_213_)begin
        if(IBusSimplePlugin_decompressor_bufferFill)begin
          IBusSimplePlugin_decompressor_bufferValid <= 1'b1;
        end
      end
      if((IBusSimplePlugin_externalFlush || IBusSimplePlugin_decompressor_consumeCurrent))begin
        IBusSimplePlugin_decompressor_throw2BytesReg <= 1'b0;
        IBusSimplePlugin_decompressor_bufferValid <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        _zz_88_ <= 1'b0;
      end
      if(IBusSimplePlugin_decompressor_output_ready)begin
        _zz_88_ <= (IBusSimplePlugin_decompressor_output_valid && (! IBusSimplePlugin_externalFlush));
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
      IBusSimplePlugin_pending_value <= IBusSimplePlugin_pending_next;
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter - _zz_279_);
      if(IBusSimplePlugin_iBusRsp_flush)begin
        IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_pending_value - _zz_281_);
      end
      _zz_120_ <= 1'b0;
      _zz_132_ <= (_zz_37_ && writeBack_arbitration_isFiring);
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
      if(_zz_214_)begin
        if(_zz_215_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_216_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_217_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
      end
      if(_zz_218_)begin
        if(_zz_219_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_220_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_221_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_222_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_223_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_224_)begin
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
      if(_zz_196_)begin
        _zz_146_ <= CsrPlugin_targetPrivilege;
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
      if(_zz_197_)begin
        case(_zz_198_)
          2'b11 : begin
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= 1'b1;
            _zz_146_ <= CsrPlugin_mstatus_MPP;
          end
          2'b01 : begin
            CsrPlugin_sstatus_SPP <= (1'b0);
            CsrPlugin_sstatus_SIE <= CsrPlugin_sstatus_SPIE;
            CsrPlugin_sstatus_SPIE <= 1'b1;
            _zz_146_ <= {(1'b0),CsrPlugin_sstatus_SPP};
          end
          default : begin
          end
        endcase
      end
      execute_CsrPlugin_wfiWake <= (({_zz_152_,{_zz_151_,{_zz_150_,{_zz_149_,{_zz_148_,_zz_147_}}}}} != 6'h0) || CsrPlugin_thirdPartyWake);
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= _zz_26_;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
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
      if(execute_CsrPlugin_csr_768)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
          CsrPlugin_mstatus_MPIE <= _zz_319_[0];
          CsrPlugin_mstatus_MIE <= _zz_320_[0];
          CsrPlugin_sstatus_SPP <= execute_CsrPlugin_writeData[8 : 8];
          CsrPlugin_sstatus_SPIE <= _zz_321_[0];
          CsrPlugin_sstatus_SIE <= _zz_322_[0];
        end
      end
      if(execute_CsrPlugin_csr_836)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sip_STIP <= _zz_324_[0];
          CsrPlugin_sip_SSIP <= _zz_325_[0];
          CsrPlugin_sip_SEIP_SOFT <= _zz_326_[0];
        end
      end
      if(execute_CsrPlugin_csr_772)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mie_MEIE <= _zz_327_[0];
          CsrPlugin_mie_MTIE <= _zz_328_[0];
          CsrPlugin_mie_MSIE <= _zz_329_[0];
          CsrPlugin_sie_SEIE <= _zz_330_[0];
          CsrPlugin_sie_STIE <= _zz_331_[0];
          CsrPlugin_sie_SSIE <= _zz_332_[0];
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
          CsrPlugin_medeleg_EU <= _zz_333_[0];
          CsrPlugin_medeleg_II <= _zz_334_[0];
          CsrPlugin_medeleg_LAF <= _zz_335_[0];
          CsrPlugin_medeleg_LPF <= _zz_336_[0];
          CsrPlugin_medeleg_LAM <= _zz_337_[0];
          CsrPlugin_medeleg_SAF <= _zz_338_[0];
          CsrPlugin_medeleg_IAF <= _zz_339_[0];
          CsrPlugin_medeleg_ES <= _zz_340_[0];
          CsrPlugin_medeleg_IPF <= _zz_341_[0];
          CsrPlugin_medeleg_SPF <= _zz_342_[0];
          CsrPlugin_medeleg_SAM <= _zz_343_[0];
          CsrPlugin_medeleg_IAM <= _zz_344_[0];
        end
      end
      if(execute_CsrPlugin_csr_771)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mideleg_SE <= _zz_345_[0];
          CsrPlugin_mideleg_ST <= _zz_346_[0];
          CsrPlugin_mideleg_SS <= _zz_347_[0];
        end
      end
      if(execute_CsrPlugin_csr_256)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sstatus_SPP <= execute_CsrPlugin_writeData[8 : 8];
          CsrPlugin_sstatus_SPIE <= _zz_348_[0];
          CsrPlugin_sstatus_SIE <= _zz_349_[0];
        end
      end
      if(execute_CsrPlugin_csr_324)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sip_STIP <= _zz_350_[0];
          CsrPlugin_sip_SSIP <= _zz_351_[0];
          CsrPlugin_sip_SEIP_SOFT <= _zz_352_[0];
        end
      end
      if(execute_CsrPlugin_csr_260)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_sie_SEIE <= _zz_353_[0];
          CsrPlugin_sie_STIE <= _zz_354_[0];
          CsrPlugin_sie_SSIE <= _zz_355_[0];
        end
      end
      if(iBus_cmd_ready)begin
        iBus_cmd_m2sPipe_rValid <= iBus_cmd_valid;
      end
      if(_zz_225_)begin
        dBus_cmd_halfPipe_regs_valid <= dBus_cmd_valid;
        dBus_cmd_halfPipe_regs_ready <= (! dBus_cmd_valid);
      end else begin
        dBus_cmd_halfPipe_regs_valid <= (! dBus_cmd_halfPipe_ready);
        dBus_cmd_halfPipe_regs_ready <= dBus_cmd_halfPipe_ready;
      end
    end
  end

  always @ (posedge clk) begin
    if(_zz_213_)begin
      IBusSimplePlugin_decompressor_bufferData <= IBusSimplePlugin_decompressor_input_payload_rsp_inst[31 : 16];
    end
    if(IBusSimplePlugin_decompressor_output_ready)begin
      _zz_89_ <= IBusSimplePlugin_decompressor_output_payload_pc;
      _zz_90_ <= IBusSimplePlugin_decompressor_output_payload_rsp_error;
      _zz_91_ <= IBusSimplePlugin_decompressor_output_payload_rsp_inst;
      _zz_92_ <= IBusSimplePlugin_decompressor_output_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_decompressor_raw;
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
    _zz_133_ <= _zz_36_[11 : 7];
    _zz_134_ <= _zz_46_;
    if((memory_MulDivIterativePlugin_div_counter_value == 6'h20))begin
      memory_MulDivIterativePlugin_div_done <= 1'b1;
    end
    if((! memory_arbitration_isStuck))begin
      memory_MulDivIterativePlugin_div_done <= 1'b0;
    end
    if(_zz_193_)begin
      if(_zz_208_)begin
        memory_MulDivIterativePlugin_rs1[31 : 0] <= memory_MulDivIterativePlugin_div_stage_0_outNumerator;
        memory_MulDivIterativePlugin_accumulator[31 : 0] <= memory_MulDivIterativePlugin_div_stage_0_outRemainder;
        if((memory_MulDivIterativePlugin_div_counter_value == 6'h20))begin
          memory_MulDivIterativePlugin_div_result <= _zz_304_[31:0];
        end
      end
    end
    if(_zz_209_)begin
      memory_MulDivIterativePlugin_accumulator <= 65'h0;
      memory_MulDivIterativePlugin_rs1 <= ((_zz_144_ ? (~ _zz_145_) : _zz_145_) + _zz_310_);
      memory_MulDivIterativePlugin_rs2 <= ((_zz_143_ ? (~ execute_RS2) : execute_RS2) + _zz_312_);
      memory_MulDivIterativePlugin_div_needRevert <= ((_zz_144_ ^ (_zz_143_ && (! execute_INSTRUCTION[13]))) && (! (((execute_RS2 == 32'h0) && execute_IS_RS2_SIGNED) && (! execute_INSTRUCTION[13]))));
    end
    CsrPlugin_mip_MEIP <= externalInterrupt;
    CsrPlugin_mip_MTIP <= timerInterrupt;
    CsrPlugin_mip_MSIP <= softwareInterrupt;
    CsrPlugin_sip_SEIP_INPUT <= externalInterruptS;
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + 64'h0000000000000001);
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + 64'h0000000000000001);
    end
    if(_zz_194_)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= (_zz_154_ ? IBusSimplePlugin_decodeExceptionPort_payload_code : decodeExceptionPort_payload_code);
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= (_zz_154_ ? IBusSimplePlugin_decodeExceptionPort_payload_badAddr : decodeExceptionPort_payload_badAddr);
    end
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= CsrPlugin_selfException_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= CsrPlugin_selfException_payload_badAddr;
    end
    if(_zz_214_)begin
      if(_zz_215_)begin
        CsrPlugin_interrupt_code <= (4'b0101);
        CsrPlugin_interrupt_targetPrivilege <= (2'b01);
      end
      if(_zz_216_)begin
        CsrPlugin_interrupt_code <= (4'b0001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b01);
      end
      if(_zz_217_)begin
        CsrPlugin_interrupt_code <= (4'b1001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b01);
      end
    end
    if(_zz_218_)begin
      if(_zz_219_)begin
        CsrPlugin_interrupt_code <= (4'b0101);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_220_)begin
        CsrPlugin_interrupt_code <= (4'b0001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_221_)begin
        CsrPlugin_interrupt_code <= (4'b1001);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_222_)begin
        CsrPlugin_interrupt_code <= (4'b0111);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_223_)begin
        CsrPlugin_interrupt_code <= (4'b0011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_224_)begin
        CsrPlugin_interrupt_code <= (4'b1011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
    end
    if(_zz_196_)begin
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
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_LL <= execute_MUL_LL;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_19_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MUL_LOW <= memory_MUL_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RVC <= decode_IS_RVC;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_LH <= execute_MUL_LH;
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
      decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_HL <= execute_MUL_HL;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_17_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PREDICTION_HAD_BRANCHED2 <= decode_PREDICTION_HAD_BRANCHED2;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_25_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
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
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MUL_HH <= execute_MUL_HH;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MUL_HH <= memory_MUL_HH;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_DIV <= decode_IS_DIV;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_IS_DIV <= execute_IS_DIV;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_14_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_CTRL <= _zz_11_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RS1_SIGNED <= decode_IS_RS1_SIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_32_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= _zz_50_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= _zz_49_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_48_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_9_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_6_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_4_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_RIGHT <= execute_SHIFT_RIGHT;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_30_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_2_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_RS2_SIGNED <= decode_IS_RS2_SIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_29_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if(((! writeBack_arbitration_isStuck) && (! CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack)))begin
      memory_to_writeBack_PC <= memory_PC;
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
        CsrPlugin_mip_MSIP <= _zz_323_[0];
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
        CsrPlugin_scause_interrupt <= _zz_356_[0];
        CsrPlugin_scause_exceptionCode <= execute_CsrPlugin_writeData[3 : 0];
      end
    end
    if(execute_CsrPlugin_csr_323)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_stval <= execute_CsrPlugin_writeData[31 : 0];
      end
    end
    if(iBus_cmd_ready)begin
      iBus_cmd_m2sPipe_rData_pc <= iBus_cmd_payload_pc;
    end
    if(_zz_225_)begin
      dBus_cmd_halfPipe_regs_payload_wr <= dBus_cmd_payload_wr;
      dBus_cmd_halfPipe_regs_payload_address <= dBus_cmd_payload_address;
      dBus_cmd_halfPipe_regs_payload_data <= dBus_cmd_payload_data;
      dBus_cmd_halfPipe_regs_payload_size <= dBus_cmd_payload_size;
    end
  end


endmodule
