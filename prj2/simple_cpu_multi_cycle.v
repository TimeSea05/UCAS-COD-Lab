`timescale 10ns / 1ns

module simple_cpu(
    input             clk,
    input             rst,

    output [31:0]     PC,
    input  [31:0]     Instruction,

    output [31:0]     Address,
    output            MemWrite,
    output [31:0]     Write_data,
    output [ 3:0]     Write_strb,

    input  [31:0]     Read_data,
    output            MemRead
);

    // THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
    // PLEASE DO NOT MODIFY SIGNAL NAMES
    // AND PLEASE USE THEM TO CONNECT PORTS
    // OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
    wire			RF_wen;
    wire [4:0]		RF_waddr;
    wire [31:0]		RF_wdata;

    // TODO: PLEASE ADD YOUR CODE BELOW
    reg [31:0] IR;
    wire [5:0] opcode = IR[31:26];

    wire [15:0] imm = IR[15:0];
    wire [31:0] signExtImm = {{16{imm[15]}}, imm[15:0]};
    wire [31:0] zeroExtImm = {16'b0, imm[15:0]};
    wire [31:0] extImm = isITypeCalcLogic ? zeroExtImm : signExtImm;

    wire [4:0] rs = IR[25:21];
    wire [4:0] rt = IR[20:16];
    wire [4:0] rd = IR[15:11];

    /********** Decoding **********/
    // NOP Instruction
    wire isNOP = IR == 32'b0;

    // R-Type Instruction
    wire isRType = (opcode == 6'b000000) & (~isNOP);
    wire [5:0] func = IR[5:0];

    // R-Type Calculation Instruction
    wire isRTypeCalc = isRType & (func[5] == 1'b1);
    wire isRTypeCalcAddorSub = isRTypeCalc & (func[3:2] == 2'b00);
    wire isRTypeCalcLogic = isRTypeCalc & (func[3:2] == 2'b01);
    wire isRTypeCalcCmp = isRTypeCalc & (func[3:2] == 2'b10);

    wire [2:0] RTypeCalcALUop;
    assign RTypeCalcALUop = {3{isRTypeCalcAddorSub}} & {func[1], 2'b10}
                          | {3{isRTypeCalcLogic}} & {func[1], 1'b0, func[0]}
                          | {3{isRTypeCalcCmp}} & {~func[0], 2'b11};

    // R-Type Shift Instruction
    wire isRTypeShift = isRType & (func[5:3] == 3'b000);
    wire isShiftOnVariable = isRTypeShift & func[2];
    wire [1:0] RTypeShiftop = func[1:0];
    
    wire [4:0] sa = IR[10:6];

    // For R-Type Jump Instructions and R-Type Mov Instructions
    // There are some cases that rd should not be written
    // So at this time we set RF_waddr = 5'b0

    // R-Type Jump Instruction
    wire isRTypeJump = isRType & ({func[5:3], func[1]} == 4'b0010);
    wire isJR = isRTypeJump & (func[2:0] == 3'b000);
    wire isJALR = isRTypeJump & (func[2:0] == 3'b001);
    wire [31:0] RTypeJumpEA = rdata1Reg;

    // R-Type Mov Instruction
    wire isRTypeMov = isRType & ({func[5:3], func[1]} == 4'b0011);
    wire isMOVZ = isRTypeMov & (func[0] == 1'b0);
    wire isMOVN = isRTypeMov & (func[0] == 1'b1);
    wire [2:0] RTypeMovALUop = 3'b110; // SUB

    wire RTypeMoveWrite = (ZeroReg & isMOVZ) | ((~ZeroReg) & isMOVN);
    
    // REGIMM Type Instruction
    wire isREGIMMType = opcode == 6'b000001;

    wire isBLTZ = isREGIMMType & (rt == 5'b00000);
    wire isBGEZ = isREGIMMType & (rt == 5'b00001);
    wire [2:0] REGIMMALUop = 3'b111;

    wire REGIMMWillBrach = (isBLTZ & (ALU_Result == 32'b1)) | (isBGEZ & ~(ALU_Result == 32'b1));

    // J-Type Instruction
    wire isJ = opcode == 6'b000010;
    wire isJAL = opcode == 6'b000011;
    wire isJType = isJ | isJAL;

    wire [25:0] instr_index = IR[25:0];
    wire [31:0] JTypeEA = {PCReg[31:28], instr_index, 2'b00};

    // I-Type Branch Instruction
    wire isITypeBranch = (opcode[5:2] == 4'b0001);

    wire isBEQ = opcode == 6'b000100;
    wire isBNE = opcode == 6'b000101;
    wire isBLEZ = opcode == 6'b000110;
    wire isBGTZ = opcode == 6'b000111;

    wire [2:0] ITypeBranchALUop = (opcode[1]) ? 3'b111 : 3'b110;
    wire ITypewillBranch = (Zero & isBEQ) | ((~Zero) & isBNE)
                         | ((~Zero | RF_rdata1 == 32'b0) & isBLEZ)
                         | ((Zero & RF_rdata1 != 32'b0) & isBGTZ);

    // I-Type Calculation Instruction
    wire isITypeCalc = opcode[5:3] == 3'b001;
    wire isLUI = opcode == 6'b001111;

    wire isITypeCalcAdd = isITypeCalc & (opcode[2:1] == 2'b00);
    wire isITypeCalcLogic = isITypeCalc & (opcode[2] == 1'b1 && opcode[1:0] != 2'b11);
    wire isITypeCalcCmp = isITypeCalc & (opcode[2:1] == 2'b01);

    wire [2:0] ITypeCalcALUop = {3{isITypeCalcAdd}} & {opcode[1], 2'b10}
                              | {3{isITypeCalcLogic & (~isLUI)}} & {opcode[1], 1'b0, opcode[0]}
                              | {3{isITypeCalcCmp}} & {~opcode[0], 2'b11};
    wire [31:0] LUIData = {imm, 16'b0};

    // I-Type Memory Access Instruction
    wire [1:0] byteDecide = originalAddr[1:0];

    // I-Type Memory Read Instruction
    wire isITypeMemRead = opcode[5:3] == 3'b100;

    wire isLB = opcode == 6'b100000;
    wire isLH = opcode == 6'b100001;
    wire isLW = opcode == 6'b100011;
    wire isLBU = opcode == 6'b100100;
    wire isLHU = opcode == 6'b100101;
    wire isLWL = opcode == 6'b100010;
    wire isLWR = opcode == 6'b100110;
    
    wire [2:0] ITypeMemReadALUop = 3'b010;

    wire [31:0] signExtendedByte = {32{byteDecide == 2'b00}} & {{24{MDR[7]}}, MDR[7:0]}
                                 | {32{byteDecide == 2'b01}} & {{24{MDR[15]}}, MDR[15:8]}
                                 | {32{byteDecide == 2'b10}} & {{24{MDR[23]}}, MDR[23:16]}
                                 | {32{byteDecide == 2'b11}} & {{24{MDR[31]}}, MDR[31:24]};

    wire [31:0] unsignExtendedByte = {32{byteDecide == 2'b00}} & {24'b0, MDR[7:0]}
                                   | {32{byteDecide == 2'b01}} & {24'b0, MDR[15:8]}
                                   | {32{byteDecide == 2'b10}} & {24'b0, MDR[23:16]}
                                   | {32{byteDecide == 2'b11}} & {24'b0, MDR[31:24]};
    
    wire [31:0] signExtendedHalf = byteDecide[1] == 1'b0 ? {{16{MDR[15]}}, MDR[15:0]}
                                 : {{16{MDR[31]}}, MDR[31:16]};

    wire [31:0] unsignExtendedHalf = byteDecide[1] == 1'b0 ? {16'b0, MDR[15:0]}
                                 : {16'b0, MDR[31:16]};
    wire [31:0] LWLData = {32{byteDecide == 2'b00}} & {MDR[7:0], rdata2Reg[23:0]}
                        | {32{byteDecide == 2'b01}} & {MDR[15:0], rdata2Reg[15:0]}
                        | {32{byteDecide == 2'b10}} & {MDR[23:0], rdata2Reg[7:0]}
                        | {32{byteDecide == 2'b11}} & MDR;
    wire [31:0] LWRData = {32{byteDecide == 2'b00}} & MDR
                        | {32{byteDecide == 2'b01}} & {rdata2Reg[31:24], MDR[31:8]}
                        | {32{byteDecide == 2'b10}} & {rdata2Reg[31:24], MDR[31:16]}
                        | {32{byteDecide == 2'b11}} & {rdata2Reg[31:8], MDR[31:24]};

    wire [31:0] ITypeMemReadRF_wdata = {32{isLB}} & signExtendedByte
                                     | {32{isLBU}} & unsignExtendedByte
                                     | {32{isLH}} & signExtendedHalf
                                     | {32{isLHU}} & unsignExtendedHalf
                                     | {32{isLW}} & MDR
                                     | {32{isLWL}} & LWLData
                                     | {32{isLWR}} & LWRData;

    // I-Type Memory Write Instruction
    wire isITypeMemWrite = opcode[5:3] == 3'b101;
    wire [2:0] ITypeMemWriteALUop = 3'b010;
    
    wire isSB = opcode == 6'b101000;
    wire isSH = opcode == 6'b101001;
    wire isSW = opcode == 6'b101011;
    wire isSWL = opcode == 6'b101010;
    wire isSWR = opcode == 6'b101110;

    wire [3:0] SB_strb = {4{byteDecide == 2'b00}} & 4'b0001
                       | {4{byteDecide == 2'b01}} & 4'b0010
                       | {4{byteDecide == 2'b10}} & 4'b0100
                       | {4{byteDecide == 2'b11}} & 4'b1000;
    wire [3:0] SH_strb = {4{byteDecide == 2'b00}} & 4'b0011
                       | {4{byteDecide == 2'b10}} & 4'b1100;
    wire [3:0] SW_strb = 4'b1111;

    wire [3:0] SWL_strb = {4{byteDecide == 2'b00}} & 4'b0001
                        | {4{byteDecide == 2'b01}} & 4'b0011
                        | {4{byteDecide == 2'b10}} & 4'b0111
                        | {4{byteDecide == 2'b11}} & 4'b1111;
    wire [3:0] SWR_strb = {4{byteDecide == 2'b00}} & 4'b1111
                        | {4{byteDecide == 2'b01}} & 4'b1110
                        | {4{byteDecide == 2'b10}} & 4'b1100
                        | {4{byteDecide == 2'b11}} & 4'b1000;

    wire [31:0] WriteByte_data = {32{byteDecide == 2'b00}} & {24'b0, rdata2Reg[7:0]}
                               | {32{byteDecide == 2'b01}} & {16'b0, rdata2Reg[7:0], 8'b0}
                               | {32{byteDecide == 2'b10}} & {8'b0, rdata2Reg[7:0], 16'b0}
                               | {32{byteDecide == 2'b11}} & {rdata2Reg[7:0], 24'b0};
    wire [31:0] WriteHalf_data = {32{byteDecide == 2'b00}} & {16'b0, rdata2Reg[15:0]}
                               | {32{byteDecide == 2'b10}} & {rdata2Reg[15:0], 16'b0};
    wire [31:0] WriteWord_data = rdata2Reg;
    wire [31:0] WriteSWL_data = {32{byteDecide == 2'b00}} & {24'b0, rdata2Reg[31:24]}
                              | {32{byteDecide == 2'b01}} & {16'b0, rdata2Reg[31:16]}
                              | {32{byteDecide == 2'b10}} & {8'b0, rdata2Reg[31:8]}
                              | {32{byteDecide == 2'b11}} & rdata2Reg[7:0];
    wire [31:0] WriteSWR_data = {32{byteDecide == 2'b00}} & rdata2Reg
                              | {32{byteDecide == 2'b01}} & {rdata2Reg[23:0], 8'b0}
                              | {32{byteDecide == 2'b10}} & {rdata2Reg[15:0], 16'b0}
                              | {32{byteDecide == 2'b11}} & {rdata2Reg[7:0], 24'b0};

    wire backToINSTR_FETCH = (isNowINSTR_DECODE & isNOP)
                           | (isNowEXUCUTE & (isREGIMMType | isITypeBranch | isJ))
                           | (isNowMEM_ACCESS & isITypeMemWrite)
                           | isNowWRITE_BACK;

    /********** My register file **********/
    wire [4:0]		RF_raddr1;
    wire [4:0]		RF_raddr2;
    wire [31:0]		RF_rdata1;
    wire [31:0]		RF_rdata2;

    reg [31:0] rdata1Reg;
    reg [31:0] rdata2Reg;

    assign RF_raddr1 = rs;
    assign RF_raddr2 = isREGIMMType ? 5'b0 : rt;
    
    // R-Type Instruction: write reg: rd
    // I-Type calc, I-Type mem read : write reg: rt
    // jal: write reg: $31
    // Other Instructions: wen = 0; waddr = 5'b0;
    assign RF_wen = (isNowEXUCUTE & (isRTypeCalc | isRTypeShift | isJALR | (isRTypeMov & RTypeMoveWrite) | isJAL | isITypeCalc))
                  | (isNowMEM_ACCESS & isITypeMemRead);
    assign RF_waddr = {5{isJAL}} & 5'b11111
                    | {5{isRTypeCalc | isRTypeShift | isJALR | RTypeMoveWrite}} & rd
                    | {5{isITypeCalc | isITypeMemRead}} & rt;

    assign RF_wdata = {32{(isRTypeCalc | (isITypeCalc & (~isLUI)) | isRTypeShift | isJAL | isJALR)}} & calcResultReg
                    | {32{RTypeMoveWrite}} & rdata1Reg
                    | {32{isLUI}} & LUIData
                    | {32{isITypeMemRead}} & ITypeMemReadRF_wdata;

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

    reg [31:0] calcResultReg;
    /********** My ALU **********/
    localparam AND  = 3'b000,
               OR   = 3'b001,
               XOR  = 3'b100,
               NOR  = 3'b101,
               ADD  = 3'b010,
               SUB  = 3'b110,
               SLT  = 3'b111,
               SLTU = 3'b011;

    wire [31:0] ALU_A;
    wire [31:0] ALU_B;
    wire [2:0] LogicALUop;
    wire [2:0] ALUop;
    wire Overflow, CarryOut, Zero;
    wire [31:0] ALU_Result;
    reg ZeroReg;

    wire [1:0] ALUSrcA = {2{isNowINIT | isNowINSTR_FETCH | backToINSTR_FETCH | isJAL | isJALR}} & 2'b00	// PC
                       | {2{isNowINSTR_DECODE & (~backToINSTR_FETCH) & isRTypeMov & (~isJAL) & (~isJALR)}} & 2'b01	// RTypeMov
                       | {2{isNowINSTR_DECODE & (~backToINSTR_FETCH) & (~isRTypeMov) & (~isJAL) & (~isJALR)}} & 2'b10;	// RF_rdata1
    assign ALU_A = {32{ALUSrcA == 2'b00}} & PCReg
                 | {32{ALUSrcA == 2'b01}} & 32'b0
                 | {32{ALUSrcA == 2'b10}} & rdata1Reg;

    wire [1:0] ALUSrcB = {2{isNowINIT | backToINSTR_FETCH | (isNowINSTR_DECODE & (isJAL | isJALR))}} & 2'b00	// 32'b4
                       | {2{isNowINSTR_DECODE & (~backToINSTR_FETCH) & (isITypeCalc | isITypeMemRead | isITypeMemWrite) & (~isJAL) & (~isJALR)}} & 2'b01	// Read from extImm
                       | {2{isNowINSTR_FETCH & (~backToINSTR_FETCH)}} & 2'b10 // Read from shifter Result(shifted extImm)
                       | {2{isNowINSTR_DECODE & (~backToINSTR_FETCH) & (~isITypeCalc & ~isITypeMemRead & ~isITypeMemWrite) & (~isJAL) & (~isJALR)}} & 2'b11; // Read from RF_rdata2

    assign ALU_B = {32{ALUSrcB == 2'b00}} & 4'h4
                 | {32{ALUSrcB == 2'b01}} & extImm
                 | {32{ALUSrcB == 2'b10}} & ShiftResult
                 | {32{ALUSrcB == 2'b11}} & rdata2Reg;

    assign LogicALUop = {3{isRTypeCalc}} & RTypeCalcALUop
                       | {3{isRTypeMov}} & RTypeMovALUop
                       | {3{isREGIMMType}} & REGIMMALUop
                       | {3{isITypeBranch}} & ITypeBranchALUop
                       | {3{isITypeCalc}} & ITypeCalcALUop
                       | {3{isITypeMemRead}} & ITypeMemReadALUop
                       | {3{isITypeMemWrite}} & ITypeMemWriteALUop;
    assign ALUop = {3{isNowINIT | isNowINSTR_FETCH | backToINSTR_FETCH | (isNowINSTR_DECODE & (isJAL | isJALR))}} & ADD
                 | {3{isNowINSTR_DECODE & (~isJAL) & (~isJALR)}} & LogicALUop;

    alu cpu_alu(
        .A(ALU_A),
        .B(ALU_B),
        .ALUop(ALUop),
        .Overflow(Overflow),
        .CarryOut(CarryOut),
        .Zero(Zero),
        .Result(ALU_Result)
    );

    /********** My Shifter **********/
    wire willBranch = REGIMMWillBrach | ITypewillBranch;
    wire isBranch = isREGIMMType | isITypeBranch;

    wire [31:0] ShifterA = isNowINSTR_FETCH ? extImm : rdata2Reg;
    wire [4:0] ShifterB = {5{isNowINSTR_FETCH}} & 5'b00010
                        | {5{isNowINSTR_DECODE & isRTypeShift & isShiftOnVariable}} & rdata1Reg[4:0]
                        | {5{isNowINSTR_DECODE & isRTypeShift & (~isShiftOnVariable)}} & sa;
    wire [1:0] Shiftop = isNowINSTR_FETCH ? 2'b00 : RTypeShiftop;
    wire [31:0] ShiftResult;

    shifter cpu_shifter(
        .A(ShifterA),
        .B(ShifterB),
        .Shiftop(Shiftop),
        .Result(ShiftResult)
    );

    /********** Memory **********/
    reg [31:0] MDR;

    assign MemWrite = (opcode[5] & opcode[3]) & isNowEXUCUTE;
    assign MemRead = opcode[5] & (~opcode[3]) & isNowEXUCUTE;
    // wire [31:0] originalAddr = {32{isNow}} & calcResultReg;
    wire [31:0] originalAddr = calcResultReg;
    assign Address = {originalAddr[31:2], 2'b00};
    assign Write_data = {32{isSB}} & WriteByte_data
                      | {32{isSH}} & WriteHalf_data
                      | {32{isSW}} & WriteWord_data
                      | {32{isSWL}} & WriteSWL_data
                      | {32{isSWR}} & WriteSWR_data;
    assign Write_strb = {4{isSB}} & SB_strb
                      | {4{isSH}} & SH_strb
                      | {4{isSW}} & SW_strb
                      | {4{isSWL}} & SWL_strb
                      | {4{isSWR}} & SWR_strb;

    /********** PC **********/
    reg [31:0] PCReg;
    assign PC = PCReg;
    wire [1:0] PC_Src;
    wire [31:0] nextPCAddress;
    assign PC_Src = {2{isNowINIT | backToINSTR_FETCH}} & 2'b00
                  | {2{(isNowINSTR_DECODE & willBranch) | isNowINSTR_FETCH}} & 2'b01 // get PC from Calc
                  | {2{isNowINSTR_DECODE & isRTypeJump}} & 2'b10 // get PC from RTypeJumpEA(rs)
                  | {2{isNowINSTR_DECODE & isJType}} & 2'b11;	// get PC from JTypeEA
    assign nextPCAddress = {32{PC_Src == 2'b00}} & ALU_Result
                         | {32{PC_Src == 2'b01}} & calcResultReg
                         | {32{PC_Src == 2'b10}} & RTypeJumpEA
                         | {32{PC_Src == 2'b11}} & JTypeEA;

    /********** Three-stage state machine **********/
    localparam INIT         = 6'b000001,
               INSTR_FETCH  = 6'b000010,
               INSTR_DECODE = 6'b000100,
               EXECUTE      = 6'b001000,
               MEM_ACCESS   = 6'b010000,
               WRITE_BACK   = 6'b100000;

    reg [5:0] current_state;
    reg [5:0] next_state;

    wire isNowINIT = current_state == INIT;
    wire isNowINSTR_FETCH = current_state == INSTR_FETCH;
    wire isNowINSTR_DECODE = current_state == INSTR_DECODE;
    wire isNowEXUCUTE = current_state == EXECUTE;
    wire isNowMEM_ACCESS = current_state == MEM_ACCESS;
    wire isNowWRITE_BACK = current_state == WRITE_BACK;

    always @(posedge clk) begin
        if (rst) current_state <= INIT;
        else current_state <= next_state;
    end

    always @(*) begin
        case (current_state)
            INIT: begin
                next_state = INSTR_FETCH;
            end

            INSTR_FETCH: begin
                next_state = INSTR_DECODE;
            end

            INSTR_DECODE: begin
                if (isNOP) next_state = INSTR_FETCH;
                else next_state = EXECUTE;
            end

            EXECUTE: begin
                if (isREGIMMType | isITypeBranch | isJ) begin
                    next_state = INSTR_FETCH;
                end
                else if (isRType | isITypeCalc | isJAL) begin
                    next_state = WRITE_BACK;
                end
                else if (isITypeMemRead | isITypeMemWrite) begin
                    next_state = MEM_ACCESS;
                end
                else begin
                    next_state = EXECUTE;
                end
            end

            MEM_ACCESS: begin
                if (isITypeMemWrite) next_state = INSTR_FETCH;
                else if (isITypeMemRead) next_state = WRITE_BACK;
                else next_state = MEM_ACCESS;
            end

            WRITE_BACK: begin
                next_state = INSTR_FETCH;
            end
            default: next_state = INIT;
        endcase
    end

    always @(posedge clk) begin
        if (backToINSTR_FETCH | isNowINIT | isNowWRITE_BACK) begin
            IR <= Instruction;
        end
    end

    always @(posedge clk) begin
        if (backToINSTR_FETCH | isNowINIT & ~(rst) | isNowWRITE_BACK 
            | (isNowINSTR_DECODE & (willBranch | isRTypeJump | isJType))) begin
                PCReg <= nextPCAddress;
            end
        else if (rst) PCReg <= 32'b0;
    end

    always @(posedge clk) begin
        if (backToINSTR_FETCH | isNowINIT | isNowINSTR_FETCH
            | (isNowINSTR_DECODE & (~isRTypeShift))) begin
                calcResultReg <= ALU_Result;
                ZeroReg <= Zero;
            end
        else if (isNowINSTR_DECODE & isRTypeShift) begin
            calcResultReg <= ShiftResult;
            ZeroReg <= Zero;
        end
    end

    always @(posedge clk) begin
        if (isNowINSTR_FETCH) begin
            rdata1Reg <= RF_rdata1;
            rdata2Reg <= RF_rdata2;
        end
    end

    always @(posedge clk) begin
        if (isNowEXUCUTE) MDR <= Read_data;
    end
endmodule
