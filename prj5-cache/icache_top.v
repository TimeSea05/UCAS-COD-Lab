`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256
`define ADDR_WIDTH  3

module icache_top (
    input	      clk,
    input	      rst,
    
    //CPU interface
    /** CPU instruction fetch request to Cache: valid signal */
    input         from_cpu_inst_req_valid,
    /** CPU instruction fetch request to Cache: address (4 byte alignment) */
    input  [31:0] from_cpu_inst_req_addr,
    /** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
    output        to_cpu_inst_req_ready,
    
    /** Cache responses to CPU: valid signal */
    output        to_cpu_cache_rsp_valid,
    /** Cache responses to CPU: 32-bit Instruction value */
    output [31:0] to_cpu_cache_rsp_data,
    /** Acknowledgement from CPU: Ready to receive Instruction */
    input	      from_cpu_cache_rsp_ready,

    //Memory interface (32 byte aligned address)
    /** Cache sending memory read request: valid signal */
    output        to_mem_rd_req_valid,
    /** Cache sending memory read request: address (32 byte alignment) */
    output [31:0] to_mem_rd_req_addr,
    /** Acknowledgement from memory: ready to receive memory read request */
    input         from_mem_rd_req_ready,

    /** Memory return read data: valid signal of one data beat */
    input         from_mem_rd_rsp_valid,
    /** Memory return read data: 32-bit one data beat */
    input  [31:0] from_mem_rd_rsp_data,
    /** Memory return read data: if current data beat is the last in this burst data transmission */
    input         from_mem_rd_rsp_last,
    /** Acknowledgement from cache: ready to receive current data beat */
    output        to_mem_rd_rsp_ready
);

    // TOD | rstO: Please add your I-Cache code here
    wire [`ADDR_WIDTH - 1:0] set = from_cpu_inst_req_addr[7:5];
    wire [23:0] tag = from_cpu_inst_req_addr[31:8];
    wire [4:0] offset = from_cpu_inst_req_addr[4:0];
    integer i;

    /********** valid array **********/
    reg [7:0] valid_array_way0;
    reg [7:0] valid_array_way1;
    reg [7:0] valid_array_way2;
    reg [7:0] valid_array_way3;

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < `CACHE_SET; i = i + 1) begin
                valid_array_way0[i] <= 1'b0;
                valid_array_way1[i] <= 1'b0;
                valid_array_way2[i] <= 1'b0;
                valid_array_way3[i] <= 1'b0;
            end
        end
        else if (isNowREFILL) begin
            if (cache_hit_way0_reg) valid_array_way0[set] <= 1'b1;
            if (cache_hit_way1_reg) valid_array_way1[set] <= 1'b1;
            if (cache_hit_way2_reg)	valid_array_way2[set] <= 1'b1;
            if (cache_hit_way3_reg)	valid_array_way3[set] <= 1'b1;
        end
    end

    /********* tag array (4 ways) **********/
    reg [23:0] tag_array_way0 [7:0];
    reg [23:0] tag_array_way1 [7:0];
    reg [23:0] tag_array_way2 [7:0];
    reg [23:0] tag_array_way3 [7:0];

    /********** data array(4 ways) **********/
    reg [255:0] data_array_way0 [7:0];
    reg [255:0] data_array_way1 [7:0];
    reg [255:0] data_array_way2 [7:0];
    reg [255:0] data_array_way3 [7:0];

    /********** write to tag_array & data_array **********/
    always @(posedge clk) begin
        if (isNowREFILL) begin
            if (cache_hit_way0_reg) begin
                tag_array_way0[set] <= tag;
                data_array_way0[set] <= new_cache_line;
            end
            else if (cache_hit_way1_reg) begin
                tag_array_way1[set] <= tag;
                data_array_way1[set] <= new_cache_line;
            end
            else if (cache_hit_way2_reg) begin
                tag_array_way2[set] <= tag;
                data_array_way2[set] <= new_cache_line;
            end
            else if (cache_hit_way3_reg) begin
                tag_array_way3[set] <= tag;
                data_array_way3[set] <= new_cache_line;
            end
        end
    end

    /********** Comparators and Cache Hit Signals **********/
    wire cache_hit_way0 = tag_array_way0[set] == tag & valid_array_way0[set];
    wire cache_hit_way1 = tag_array_way1[set] == tag & valid_array_way1[set];
    wire cache_hit_way2 = tag_array_way2[set] == tag & valid_array_way2[set];
    wire cache_hit_way3 = tag_array_way3[set] == tag & valid_array_way3[set];

    reg cache_hit_way1_reg;
    reg cache_hit_way0_reg;
    reg cache_hit_way2_reg;
    reg cache_hit_way3_reg;

    wire cache_hit = cache_hit_way0 | cache_hit_way1 | cache_hit_way2 | cache_hit_way3;
    wire all_valid = valid_array_way0[set] & valid_array_way1[set]
                   & valid_array_way2[set] & valid_array_way3[set];

    /********** read data from cache **********/
    wire [`LINE_LEN - 1:0] cache_data = {256{cache_hit_way0}} & data_array_way0[set]
                                      | {256{cache_hit_way1}} & data_array_way1[set]
                                      | {256{cache_hit_way2}} & data_array_way2[set]
                                      | {256{cache_hit_way3}} & data_array_way3[set];

    wire [31:0] cache_hit_data = cache_data[{offset, 3'b0}+:32];
    assign to_cpu_cache_rsp_data = cache_hit_data;

    /********** PLRU Algorithm **********/
    reg [2:0] PLRU_reg [`CACHE_SET - 1:0];

    wire conflict_hit_way0 = all_valid & ~PLRU_reg[set][0] & ~PLRU_reg[set][1]
                             | ~valid_array_way0[set];
    wire conflict_hit_way1 = all_valid & ~PLRU_reg[set][0] & PLRU_reg[set][1]
                             | valid_array_way0[set] & ~valid_array_way1[set];
    wire conflict_hit_way2 = all_valid & PLRU_reg[set][0] & ~PLRU_reg[set][2]
                             | valid_array_way0[set] & valid_array_way1[set] & ~valid_array_way2[set];
    wire conflict_hit_way3 = all_valid & PLRU_reg[set][0] & PLRU_reg[set][2]
                             | valid_array_way0[set] & valid_array_way1[set] & valid_array_way2[set] & ~valid_array_way3[set];
    
    always @(posedge clk) begin
        if (isNowTAG_READ) begin
            cache_hit_way0_reg <= cache_hit_way0;
            cache_hit_way1_reg <= cache_hit_way1;
            cache_hit_way2_reg <= cache_hit_way2;
            cache_hit_way3_reg <= cache_hit_way3;
        end
        else if (isNowEVICT) begin
            cache_hit_way0_reg <= conflict_hit_way0;
            cache_hit_way1_reg <= conflict_hit_way1;
            cache_hit_way2_reg <= conflict_hit_way2;
            cache_hit_way3_reg <= conflict_hit_way3;
        end
    end

    wire cache_hit_way01 = cache_hit_way0_reg | cache_hit_way1_reg;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < `CACHE_SET; i = i + 1) begin
                PLRU_reg[i] <= 3'b0;
            end
        end
        else if (isNowRESPONSE) begin
            PLRU_reg[set][0] <= cache_hit_way01;
            if (cache_hit_way01) PLRU_reg[set][1] <= cache_hit_way0_reg;
            if (~cache_hit_way01) PLRU_reg[set][2] <= cache_hit_way2_reg;
        end
    end


    /********** state machine **********/
    reg [7:0] current_state;
    reg [7:0] next_state;

    localparam CPU_WAIT   = 8'b00000001,
               TAG_READ   = 8'b00000010,
               CACHE_READ = 8'b00000100,
               EVICT      = 8'b00001000,
               MEM_READ   = 8'b00010000,
               RECEIVE    = 8'b00100000,
               REFILL     = 8'b01000000,
               RESPONSE   = 8'b10000000;
    
    always @(posedge clk) begin
        if (rst) current_state <= CPU_WAIT;
        else current_state <= next_state;
    end

    always @(*) begin
        case (current_state)
            CPU_WAIT: begin
                if (from_cpu_inst_req_valid) next_state = TAG_READ;
                else next_state = CPU_WAIT;
            end

            TAG_READ: begin
                if (cache_hit) next_state = CACHE_READ;
                else next_state = EVICT;
            end

            CACHE_READ: next_state = RESPONSE;

            EVICT: next_state = MEM_READ;

            MEM_READ: begin
                if (from_mem_rd_req_ready) next_state = RECEIVE;
                else next_state = MEM_READ;
            end

            RECEIVE: begin
                if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) begin
                    next_state = REFILL;
                end
                else next_state = RECEIVE;
            end

            REFILL: next_state = RESPONSE;

            RESPONSE: begin
                if (from_cpu_cache_rsp_ready) next_state = CPU_WAIT;
                else next_state = RESPONSE;
            end

            default: next_state = CPU_WAIT;
        endcase
    end

    wire isNowWAIT_CPU   = current_state == CPU_WAIT;
    wire isNowTAG_READ   = current_state == TAG_READ;
    wire isNowCACHE_READ = current_state == CACHE_READ;
    wire isNowEVICT      = current_state == EVICT;
    wire isNowMEM_READ   = current_state == MEM_READ;
    wire isNowRECEIVE    = current_state == RECEIVE;
    wire isNowREFILL     = current_state == REFILL;
    wire isNowRESPONSE   = current_state == RESPONSE;

    /********** deal with handshake signals **********/
    assign to_cpu_inst_req_ready = isNowWAIT_CPU;
    assign to_cpu_cache_rsp_valid = isNowRESPONSE;
    assign to_mem_rd_req_valid = isNowMEM_READ;
    assign to_mem_rd_rsp_ready = isNowRECEIVE | rst;

    /********** store signals from cpu **********/
    reg [31:0] from_cpu_inst_req_addr_reg;
    always @(posedge clk) begin
        if (isNowWAIT_CPU) from_cpu_inst_req_addr_reg <= from_cpu_inst_req_addr;
    end
    assign to_mem_rd_req_addr = {from_cpu_inst_req_addr_reg[31:5], 5'b0};

    /********** burst **********/
    reg [`LINE_LEN - 1:0] new_cache_line;
    always @(posedge clk) begin
        if (isNowRECEIVE & from_mem_rd_rsp_valid) begin
            new_cache_line <= {from_mem_rd_rsp_data, new_cache_line[`LINE_LEN - 1:32]};
        end
    end

endmodule