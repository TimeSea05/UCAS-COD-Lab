`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
    input  [`DATA_WIDTH - 1:0]  A,
    input  [`DATA_WIDTH - 1:0]  B,
    input  [              2:0]  ALUop,
    output                      Overflow,
    output                      CarryOut,
    output                      Zero,
    output [`DATA_WIDTH - 1:0]  Result
);
    // TODO: Please add your logic design here
    wire [`DATA_WIDTH:0] extendedA;
    wire [`DATA_WIDTH:0] extendedB;
    assign extendedA = {1'b0, A};
    assign extendedB = {1'b0, B};

    wire [`DATA_WIDTH - 1:0] AandB;
    wire [`DATA_WIDTH - 1:0] AorB;
    wire [`DATA_WIDTH - 1:0] AxorB;
    wire [`DATA_WIDTH - 1:0] AnorB;

    wire [`DATA_WIDTH - 1:0] AsltB;
    wire [`DATA_WIDTH - 1:0] AsltuB;

    assign AandB = A & B;
    assign AorB = A | B;
    assign AxorB = A ^ B;
    assign AnorB = ~AorB;

    wire [`DATA_WIDTH:0] extendedBForAdder;
    wire [`DATA_WIDTH - 1:0] adderResult;

    wire Binvert = (ALUop == 3'b110 || ALUop == 3'b111 || ALUop == 3'b011);

    assign extendedBForAdder = Binvert ? ~extendedB : extendedB; 
    assign {CarryOut, adderResult} = extendedA + extendedBForAdder + Binvert;

    assign Overflow = !(extendedA[`DATA_WIDTH - 1] ^ extendedBForAdder[`DATA_WIDTH - 1]) && (extendedA[`DATA_WIDTH - 1] ^ adderResult[`DATA_WIDTH - 1]);
    
    wire isDifferentSign;
    assign isDifferentSign = A[`DATA_WIDTH - 1] ^ B[`DATA_WIDTH - 1];
    assign AsltB = isDifferentSign ? 
                    {32{A[`DATA_WIDTH - 1] == 1'b1 && B[`DATA_WIDTH - 1] == 1'b0}} & 32'b1
                  | {32{A[`DATA_WIDTH - 1] == 1'b0 && B[`DATA_WIDTH - 1] == 1'b1}} & 32'b0
                 :  {32{adderResult[`DATA_WIDTH - 1] == 1'b1}} & 32'b1
                  | {32{adderResult[`DATA_WIDTH - 1] == 1'b0}} & 32'b0;
    assign AsltuB = (CarryOut == 1'b0) ? 32'b0 : 32'b1;
    
    // the final selector
    wire isAnd, isOr, isAddOrSub, isCmp, isXor, isNor, isSltu;

    assign isAnd = ALUop == 3'b000;
    assign isOr = ALUop == 3'b001;
    assign isAddOrSub = (ALUop == 3'b010) || (ALUop == 3'b110);
    assign isCmp = ALUop == 3'b111;
    assign isXor = ALUop == 3'b100;
    assign isNor = ALUop == 3'b101;
    assign isSltu = ALUop == 3'b011;

    assign Result = {32{isAnd}} & AandB
                  | {32{isOr}} & AorB
                  | {32{isAddOrSub}} & adderResult
                  | {32{isCmp}} & AsltB
                  | {32{isXor}} & AxorB
                  | {32{isNor}} & AnorB
                  | {32{isSltu}} & AsltuB;

    assign Zero = (Result == 32'b0);
endmodule