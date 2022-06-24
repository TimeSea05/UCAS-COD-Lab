`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
    input                       clk,
    input  [`ADDR_WIDTH - 1:0]  waddr,
    input  [`ADDR_WIDTH - 1:0]  raddr1,
    input  [`ADDR_WIDTH - 1:0]  raddr2,
    input                       wen,
    input  [`DATA_WIDTH - 1:0]  wdata,
    output [`DATA_WIDTH - 1:0]  rdata1,
    output [`DATA_WIDTH - 1:0]  rdata2
);

    // TODO: Please add your logic design here

    // register pile `mem`
    reg [`DATA_WIDTH - 1:0] mem[`DATA_WIDTH - 1:0];

    always @(posedge clk) begin
        // IMPORTANT: '$zero' register is unwritable and its value should always be 0
        if (wen == 1'b1 && waddr != 5'b0) begin
            mem[waddr] <= wdata;
        end
    end

    assign rdata1 = (raddr1 == 5'b0) ? 5'b0 : mem[raddr1];
    assign rdata2 = (raddr2 == 5'b0) ? 5'b0 : mem[raddr2];
endmodule
