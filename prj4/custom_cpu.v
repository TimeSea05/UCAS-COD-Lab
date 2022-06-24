`timescale 10ns / 1ns

module custom_cpu(
    input         clk,
    input         rst,

    //Instruction request channel
    output [31:0] PC,
    output        Inst_Req_Valid,
    input         Inst_Req_Ready,

    //Instruction response channel
    input  [31:0] Instruction,
    input         Inst_Valid,
    output        Inst_Ready,

    //Memory request channel
    output [31:0] Address,
    output        MemWrite,
    output [31:0] Write_data,
    output [ 3:0] Write_strb,
    output        MemRead,
    input         Mem_Req_Ready,

    //Memory data response channel
    input  [31:0] Read_data,
    input         Read_data_Valid,
    output        Read_data_Ready,

    input         intr,

    output [31:0] cpu_perf_cnt_0,
    output [31:0] cpu_perf_cnt_1,
    output [31:0] cpu_perf_cnt_2,
    output [31:0] cpu_perf_cnt_3,
    output [31:0] cpu_perf_cnt_4,
    output [31:0] cpu_perf_cnt_5,
    output [31:0] cpu_perf_cnt_6,
    output [31:0] cpu_perf_cnt_7,
    output [31:0] cpu_perf_cnt_8,
    output [31:0] cpu_perf_cnt_9,
    output [31:0] cpu_perf_cnt_10,
    output [31:0] cpu_perf_cnt_11,
    output [31:0] cpu_perf_cnt_12,
    output [31:0] cpu_perf_cnt_13,
    output [31:0] cpu_perf_cnt_14,
    output [31:0] cpu_perf_cnt_15,

    output [69:0] inst_retire
);

    /* The following signal is leveraged for behavioral simulation, 
    * which is delivered to testbench.
    *
    * STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
    *
    * inst_retired (70-bit): detailed information of the retired instruction,
    * mainly including (in order) 
    * { 
    *   reg_file write-back enable  (69:69,  1-bit),
    *   reg_file write-back address (68:64,  5-bit), 
    *   reg_file write-back data    (63:32, 32-bit),  
    *   retired PC                  (31: 0, 32-bit)
    * }
    *
    */
    wire [69:0] inst_retire;
    assign inst_retire = {RF_wen, RF_waddr, RF_wdata, PC};

    // TODO: Please add your custom CPU code here
    // Important registers
    reg [31:0] IR;

    wire [6:0] opcode = IR[6:0];
    wire [2:0] funct3 = IR[14:12];
    wire [6:0] funct7 = IR[31:25];

    /********** Decoding **********/
    localparam AND  = 3'b000,
               OR   = 3'b001,
               ADD  = 3'b010,
               SLTU = 3'b011,
               XOR  = 3'b100,
               NOR  = 3'b101,
               SUB  = 3'b110,
               SLT  = 3'b111;

    // R-Type Instructions
    wire isRType = opcode == 7'b0110011;
    wire isRTypeShift = ((funct3 == 3'b001) | (funct3 == 3'b101)) & isRType;
    wire isRTypeCalc = ~isRTypeShift & isRType;

    wire [4:0] rs1 = IR[19:15];
    wire [4:0] rs2 = IR[24:20];
    wire [4:0] rd = IR[11:7];

    wire isADD  = isRTypeCalc & (funct3 == 3'b000) & ~funct7[5];
    wire isSUB  = isRTypeCalc & (funct3 == 3'b000) & funct7[5];
    wire isSLT  = isRTypeCalc & (funct3 == 3'b010);
    wire isSLTU = isRTypeCalc & (funct3 == 3'b011);
    wire isXOR  = isRTypeCalc & (funct3 == 3'b100);
    wire isOR   = isRTypeCalc & (funct3 == 3'b110);
    wire isAND  = isRTypeCalc & (funct3 == 3'b111);	
    
    wire [2:0] RTypeALUop = {3{isADD}}  & ADD
                          | {3{isSUB}}  & SUB
                          | {3{isSLT}}  & SLT
                          | {3{isSLTU}} & SLTU
                          | {3{isAND}}  & AND
                          | {3{isOR}}   & OR
                          | {3{isXOR}}  & XOR;
    wire isSLL = isRTypeShift & (funct3 == 3'b001);
    wire isSRL = isRTypeShift & (funct3 == 3'b101) & ~funct7[5];
    wire isSRA = isRTypeShift & (funct3 == 3'b101) & funct7[5];

    // I-Type Instructions
    wire isIType = opcode == 7'b0010011;
    wire isITypeShift = ((funct3 == 3'b001) | (funct3 == 3'b101)) & isIType;
    wire isITypeCalc = ~isITypeShift & isIType;
    
    wire [11:0] ITypeImm = IR[31:20];
    wire [31:0] ITypeExtImm = {{20{ITypeImm[11]}}, ITypeImm};
    wire [4:0] shamt = IR[24:20];

    wire isADDI  = isITypeCalc & (funct3 == 3'b000);
    wire isSLTI  = isITypeCalc & (funct3 == 3'b010);
    wire isSLTUI = isITypeCalc & (funct3 == 3'b011);
    wire isXORI  = isITypeCalc & (funct3 == 3'b100);
    wire isORI   = isITypeCalc & (funct3 == 3'b110);
    wire isANDI  = isITypeCalc & (funct3 == 3'b111);

    wire [2:0] ITypeCalcALUop = {3{isADDI}} & ADD
                              | {3{isSLTI}} & SLT
                                | {3{isSLTUI}} & SLTU
                                | {3{isANDI}} & AND
                                | {3{isORI}} & OR
                                | {3{isXORI}} & XOR;
    wire isSLLI = isITypeShift & (funct3 == 3'b001);
    wire isSRLI = isITypeShift & (funct3 == 3'b101) & ~funct7[5];
    wire isSRAI = isITypeShift & (funct3 == 3'b101) & funct7[5];

    // Memory Access Instructions
    wire [1:0] byteDecide = originalAddr[1:0];
    // I-Type Load Instrucitons
    wire isITypeLoad = opcode == 7'b0000011;
    wire [2:0] ITypeLoadALUop = ADD;
 
    wire isLB  = isITypeLoad & (funct3 == 3'b000);
    wire isLH  = isITypeLoad & (funct3 == 3'b001);
    wire isLW  = isITypeLoad & (funct3 == 3'b010);
    wire isLBU = isITypeLoad & (funct3 == 3'b100);
    wire isLHU = isITypeLoad & (funct3 == 3'b101);

    wire [31:0] LBData  = {32{byteDecide == 2'b00}} & {{24{MDR[7]}}, MDR[7:0]}
                        | {32{byteDecide == 2'b01}} & {{24{MDR[15]}}, MDR[15:8]}
                        | {32{byteDecide == 2'b10}} & {{24{MDR[23]}}, MDR[23:16]}
                        | {32{byteDecide == 2'b11}} & {{24{MDR[31]}}, MDR[31:24]};
    wire [31:0] LHData  = byteDecide[1] == 1'b0 ? {{16{MDR[15]}}, MDR[15:0]}
                        : {{16{MDR[31]}}, MDR[31:16]};
    wire [31:0] LBUData = {32{byteDecide == 2'b00}} & {24'b0, MDR[7:0]}
                        | {32{byteDecide == 2'b01}} & {24'b0, MDR[15:8]}
                        | {32{byteDecide == 2'b10}} & {24'b0, MDR[23:16]}
                        | {32{byteDecide == 2'b11}} & {24'b0, MDR[31:24]};
    wire [31:0] LHUData = byteDecide[1] == 1'b0 ? {16'b0, MDR[15:0]}
                        : {16'b0, MDR[31:16]};
    wire [31:0] LWData  = MDR;

    wire [31:0] LoadData = {32{isLB}}  & LBData
                         | {32{isLH}}  & LHData
                         | {32{isLBU}} & LBUData
                         | {32{isLHU}} & LHUData
                         | {32{isLW}}  & LWData;

    // S-Type Write Instructions
    wire isSType = opcode == 7'b0100011;
    wire [2:0] STypeALUop = ADD;

    wire isSB = isSType & (funct3 == 3'b000);
    wire isSH = isSType & (funct3 == 3'b001);
    wire isSW = isSType & (funct3 == 3'b010);

    wire [11:0] STypeImm = {IR[31:25], IR[11:7]};
    wire [31:0] STypeExtImm = {{20{STypeImm[11]}}, STypeImm};

    wire [3:0] SB_strb = {4{byteDecide == 2'b00}} & 4'b0001
                       | {4{byteDecide == 2'b01}} & 4'b0010
                       | {4{byteDecide == 2'b10}} & 4'b0100
                       | {4{byteDecide == 2'b11}} & 4'b1000;
    wire [3:0] SH_strb = {4{byteDecide == 2'b00}} & 4'b0011
                       | {4{byteDecide == 2'b10}} & 4'b1100;
    wire [3:0] SW_strb = 4'b1111;

    wire [31:0] SBData = {32{byteDecide == 2'b00}} & {24'b0, rdata2_Reg[7:0]}
                       | {32{byteDecide == 2'b01}} & {16'b0, rdata2_Reg[7:0], 8'b0}
                       | {32{byteDecide == 2'b10}} & {8'b0, rdata2_Reg[7:0], 16'b0}
                       | {32{byteDecide == 2'b11}} & {rdata2_Reg[7:0], 24'b0};
    wire [31:0] SHData = {32{byteDecide == 2'b00}} & {16'b0, rdata2_Reg[15:0]}
                       | {32{byteDecide == 2'b10}} & {rdata2_Reg[15:0], 16'b0};
    wire [31:0] SWData = rdata2_Reg;


    // B-Type Branch Instructions
    wire isBType = opcode == 7'b1100011;
    wire isBEQ  = isBType & (funct3 == 3'b000);
    wire isBNE  = isBType & (funct3 == 3'b001);
    wire isBLT  = isBType & (funct3 == 3'b100);
    wire isBGE  = isBType & (funct3 == 3'b101);
    wire isBLTU = isBType & (funct3 == 3'b110);
    wire isBGEU = isBType & (funct3 == 3'b111);

    wire [12:0] BTypeImm = {IR[31], IR[7], IR[30:25], IR[11:8], 1'b0};
    wire [31:0] BTypeExtImm = {{19{BTypeImm[12]}}, BTypeImm};

    wire [2:0] BTypeALUop = {3{isBEQ  | isBNE }} & SUB
                          | {3{isBLT  | isBGE }} & SLT
                          | {3{isBLTU | isBGEU}} & SLTU;
    wire willBranch = (Zero & isBEQ) | (~Zero & isBNE)
                    | (ALU_Result == 32'b1 & (isBLT | isBLTU))
                    | (ALU_Result == 32'b0 & (isBGE | isBGEU));

    // U-Type Instructions
    wire isLUI   = opcode == 7'b0110111;
    wire isAUIPC = opcode == 7'b0010111;
    wire isUType = isLUI | isAUIPC;

    wire [2:0] AUIPC_ALUop = ADD;
    wire [31:0] UTypeImm = {IR[31:12], 12'b0};
    
    // J-Type Instructions
    wire isJAL  = opcode == 7'b1101111;
    wire isJALR = opcode == 7'b1100111;
    wire isJType = isJAL | isJALR;
    wire [2:0] JTypeALUop = ADD;

    wire [20:0] JALImm = {IR[31], IR[19:12], IR[20], IR[30:21], 1'b0};
    wire [31:0] JALExtImm = {{12{JALImm[19]}}, JALImm}; 
    // For JALR Instruction, just use ITypeImm and ITypeExtImm instead
    wire [31:0] JALRExtImm = ITypeExtImm;

    wire backToINSTR_FETCH = (isNowEXUCUTE & isBType)
                           | (isNowMEM_WRITE & Mem_Req_Ready)
                           | isNowWRITE_BACK;

    /********** register file **********/
    wire [4:0]		RF_raddr1;
    wire [4:0]		RF_raddr2;
    wire [31:0]		RF_rdata1;
    wire [31:0]		RF_rdata2;

    assign RF_raddr1 = rs1;
    assign RF_raddr2 = rs2;

    // wire RF_wen = inst_retire[69];
    // wire [4:0]  RF_waddr = inst_retire[68:64];
    // wire [31:0] RF_wdata = inst_retire[63:32];

    reg [31:0] rdata1_Reg;
    reg [31:0] rdata2_Reg;

    wire RF_wen   = isNowWRITE_BACK;
    wire [4:0] RF_waddr = rd;
    wire [31:0] RF_wdata = {32{isRType | isIType | isAUIPC}} & calcResult_Reg
                    | {32{isJType}} & nextInstrAddr_Reg
                    | {32{isLUI}} & UTypeImm
                    | {32{isITypeLoad}} & LoadData;

    reg_file cpu_reg_file(
        .clk(clk),
        .waddr(RF_waddr),
        .raddr1(RF_raddr1),
        .raddr2(RF_raddr2),
        .wen(RF_wen),
        .wdata(RF_wdata),
        .rdata1(RF_rdata1),
        .rdata2(RF_rdata2)
    );

    /********** ALU & shifter **********/
    reg [31:0] calcResult_Reg;
    reg [31:0] nextInstrAddr_Reg;
    reg Zero_Reg;

    wire [31:0] ALU_A;
    wire [31:0] ALU_B;
    wire [2:0] LogicALUop;
    wire [2:0] ALUop;
    wire [31:0] ALU_Result;
    wire Overflow, Carryout, Zero;

    assign ALU_A = {32{isNowINIT | isNowINSTR_FETCH | isNowINSTR_WAIT | isNowINSTR_DECODE}} & PC_Reg
                  | {32{isNowEXUCUTE}} & rdata1_Reg;
    assign ALU_B = {32{isNowINIT | isNowINSTR_FETCH | isNowINSTR_WAIT}} & 32'h4
                 | {32{isNowINSTR_DECODE & isBType}} & BTypeExtImm
                 | {32{isNowINSTR_DECODE & isJAL}} & JALExtImm
                 | {32{isNowINSTR_DECODE & isAUIPC}} & UTypeImm
                 | {32{isNowEXUCUTE & (isRTypeCalc | isBType)}} & rdata2_Reg
                 | {32{isNowEXUCUTE & (isITypeCalc | isITypeLoad)}} & ITypeExtImm
                 | {32{isNowEXUCUTE & isJALR}} & JALRExtImm
                 | {32{isNowEXUCUTE & isSType}} & STypeExtImm;

    assign LogicALUop = {3{isRTypeCalc}} & RTypeALUop
                      | {3{isITypeCalc}} & ITypeCalcALUop
                      | {3{isITypeLoad}} & ITypeLoadALUop
                      | {3{isSType}} & STypeALUop
                      | {3{isBType}} & BTypeALUop
                      | {3{isJType}} & JTypeALUop
                      | {3{isAUIPC}} & AUIPC_ALUop;
    assign ALUop = {3{isNowINIT | isNowINSTR_FETCH | isNowINSTR_WAIT | isNowINSTR_DECODE}} & ADD
                 | {3{isNowEXUCUTE}} & LogicALUop;
    
    alu cpu_alu(
        .A(ALU_A),
        .B(ALU_B),
        .ALUop(ALUop),
        .Overflow(Overflow),
        .CarryOut(CarryOut),
        .Zero(Zero),
        .Result(ALU_Result)
    );

    wire [31:0] shifterA = rdata1_Reg;
    wire [4:0]  shifterB = {5{isRTypeShift}} & rdata2_Reg[4:0]
                         | {5{isITypeShift}} & shamt;
    wire [1:0] Shiftop = {2{isSLL | isSLLI}} & 2'b00
                         | {2{isSRL | isSRLI}} & 2'b10
                         | {2{isSRA | isSRAI}} & 2'b11;
    wire [31:0] ShiftResult;

    shifter cpu_shifter(
        .A(shifterA),
        .B(shifterB),
        .Shiftop(Shiftop),
        .Result(ShiftResult)
    );

    /********** Memory **********/
    reg [31:0] MDR;
    wire [31:0] originalAddr = calcResult_Reg;
    assign Address = {originalAddr[31:1], 1'b0};

    assign MemRead = isITypeLoad & isNowMEM_READ;
    assign MemWrite = isSType & isNowMEM_WRITE;

    assign Write_strb = {4{isSB}} & SB_strb
                      | {4{isSH}} & SH_strb
                      | {4{isSW}} & SW_strb;
    assign Write_data = {32{isSB}} & SBData
                      | {32{isSH}} & SHData
                      | {32{isSW}} & SWData;

    /********** PC **********/
    reg [31:0] PC_Reg;
    assign PC = PC_Reg;
    assign inst_retire[31:0] = PC_Reg;

    wire [31:0] nextPCAddr = {32{isNowEXUCUTE & (willBranch | isJAL | isAUIPC)}} & calcResult_Reg
                           | {32{isNowEXUCUTE & isJALR}} & ALU_Result
                           | {32{isNowEXUCUTE & ~willBranch & ~isJType & ~isAUIPC}} & nextInstrAddr_Reg;

    /********** Three-stage state machine **********/
    localparam INIT          = 9'b000000001,
               INSTR_FETCH   = 9'b000000010,
               INSTR_WAIT	 = 9'b000000100,
               INSTR_DECODE  = 9'b000001000,
               EXECUTE       = 9'b000010000,
               MEM_READ      = 9'b000100000,
               MEM_READ_WAIT = 9'b001000000,
               MEM_WRITE	 = 9'b010000000,
               WRITE_BACK    = 9'b100000000;

    reg [8:0] current_state;
    reg [8:0] next_state;

    wire isNowINIT          = current_state == INIT;
    wire isNowINSTR_FETCH   = current_state == INSTR_FETCH;
    wire isNowINSTR_WAIT    = current_state == INSTR_WAIT;
    wire isNowINSTR_DECODE  = current_state == INSTR_DECODE;
    wire isNowEXUCUTE       = current_state == EXECUTE;
    wire isNowMEM_READ      = current_state == MEM_READ;
    wire isNowMEM_READ_WAIT = current_state == MEM_READ_WAIT;
    wire isNowMEM_WRITE     = current_state == MEM_WRITE;
    wire isNowWRITE_BACK    = current_state == WRITE_BACK;

    always @(posedge clk) begin
        if (rst) current_state <= INIT;
        else current_state <= next_state;
    end

    always @(*) begin
        case(current_state)
            INIT: next_state = INSTR_FETCH;

            INSTR_FETCH: begin
                if (Inst_Req_Ready) next_state = INSTR_WAIT;
                else next_state = INSTR_FETCH;
            end

            INSTR_WAIT: begin
                if (Inst_Valid) next_state = INSTR_DECODE;
                else next_state = INSTR_WAIT;
            end

            INSTR_DECODE: next_state = EXECUTE;

            EXECUTE: begin
                if (isBType) next_state = INSTR_FETCH;
                else if (isRType | isITypeCalc | isITypeShift | isUType | isJType) begin
                    next_state = WRITE_BACK;
                end
                else if (isITypeLoad) next_state = MEM_READ;
                else if (isSType) next_state = MEM_WRITE;
            end

            MEM_READ: begin
                if (Mem_Req_Ready) next_state = MEM_READ_WAIT;
                else next_state = MEM_READ;
            end

            MEM_READ_WAIT: begin
                if (Read_data_Valid) next_state = WRITE_BACK;
                else next_state = MEM_READ_WAIT;
            end
            
            MEM_WRITE: begin
                if (Mem_Req_Ready) next_state = INSTR_FETCH;
                else next_state = MEM_WRITE;
            end

            WRITE_BACK: next_state = INSTR_FETCH;

            default: next_state = INIT;
        endcase
    end

    always @(posedge clk) begin
        if (rst) IR <= 32'b0;
        else if (isNowINSTR_WAIT & Inst_Valid) begin
            IR <= Instruction;
        end
    end

    always @(posedge clk) begin
        if (rst) PC_Reg <= 32'b0;
        else if (isNowEXUCUTE) PC_Reg <= nextPCAddr;
    end

    always @(posedge clk) begin
        if (isNowINIT | isNowINSTR_FETCH | isNowINSTR_WAIT | isNowINSTR_DECODE
            | (isNowEXUCUTE & ~isRTypeShift & ~isITypeShift)) begin
                calcResult_Reg <= ALU_Result;
                Zero_Reg <= Zero;
            end
        else if (isNowEXUCUTE & (isRTypeShift | isITypeShift)) begin
            calcResult_Reg <= ShiftResult;
            Zero_Reg <= Zero;
        end
    end

    always @(posedge clk) begin
        if (isNowINIT | isNowINSTR_FETCH | isNowINSTR_WAIT) begin
            nextInstrAddr_Reg <= ALU_Result;			
        end
    end

    always @(posedge clk) begin
        if (isNowINSTR_DECODE) begin
            rdata1_Reg <= RF_rdata1;
            rdata2_Reg <= RF_rdata2;
        end
    end

    always @(posedge clk) begin
        if (isNowMEM_READ_WAIT & Read_data_Valid) MDR <= Read_data;
    end

    /********** Deal With Handshake signals **********/
    reg Inst_Req_Valid_Reg;
    reg Inst_Ready_Reg;
    reg Read_data_Ready_Reg;

    assign Inst_Req_Valid = Inst_Req_Valid_Reg;
    assign Inst_Ready = Inst_Ready_Reg;
    assign Read_data_Ready = Read_data_Ready_Reg;

    always @(posedge clk) begin
        if (rst) Inst_Req_Valid_Reg <= 1'b0;
        else if (isNowINIT | backToINSTR_FETCH | (isNowINSTR_FETCH & ~Inst_Req_Ready)) begin
            Inst_Req_Valid_Reg <= 1'b1;
        end
        else Inst_Req_Valid_Reg <= 1'b0;
    end

    always @(posedge clk) begin
        if (rst) Inst_Ready_Reg <= 1'b1;
        else if ((isNowINSTR_FETCH & Inst_Req_Ready) | (isNowINSTR_WAIT & ~Inst_Valid)) begin
            Inst_Ready_Reg <= 1'b1;
        end
        else Inst_Ready_Reg <= 1'b0;
    end

    always @(posedge clk) begin
        if (rst) Read_data_Ready_Reg <= 1'b1;
        else if ((isNowMEM_READ & Mem_Req_Ready) | (isNowMEM_READ_WAIT & ~Read_data_Valid)) begin
            Read_data_Ready_Reg <= 1'b1;
        end
        else Read_data_Ready_Reg <= 1'b0;
    end

    /********** Performance Counter **********/
    assign cpu_perf_cnt_0 = cycle_cnt;
    reg [31:0] cycle_cnt;
    always @(posedge clk) begin
        if (rst == 1'b1) cycle_cnt <= 32'b0;
        else cycle_cnt <= cycle_cnt + 32'b1;
    end
endmodule
