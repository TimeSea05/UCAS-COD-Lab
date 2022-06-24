`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
    input  [`DATA_WIDTH - 1:0] A,
    input  [              4:0] B,
    input  [              1:0] Shiftop,
    output [`DATA_WIDTH - 1:0] Result
);
    // TODO: Please add your logic code here

    wire isLeft = Shiftop == 2'b00;
    wire isLogicRight = Shiftop == 2'b10;
    wire isArithRight = Shiftop == 2'b11;
    
    wire [31:0] AleftB = A << B;
    wire [31:0] AlogicRightB = A >> B;
    wire [31:0] AarithRightB = ({{31{A[31]}}, 1'b0} << (~B[4:0])) | (A >> B[4:0]);

    assign Result = {32{isLeft}} & AleftB
                  | {32{isLogicRight}} & AlogicRightB
                  | {32{isArithRight}} & AarithRightB;
endmodule
