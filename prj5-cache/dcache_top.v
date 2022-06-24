`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256
`define ADDR_WIDTH  3

module dcache_top (
    input	      clk,
    input	      rst,
  
    //CPU interface
    /** CPU memory/IO access request to Cache: valid signal */
    input         from_cpu_mem_req_valid,
    /** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
    input         from_cpu_mem_req,
    /** CPU memory/IO access request to Cache: address (4 byte alignment) */
    input  [31:0] from_cpu_mem_req_addr,
    /** CPU memory/IO access request to Cache: 32-bit write data */
    input  [31:0] from_cpu_mem_req_wdata,
    /** CPU memory/IO access request to Cache: 4-bit write strobe */
    input  [ 3:0] from_cpu_mem_req_wstrb,
    /** Acknowledgement from Cache: ready to receive CPU memory access request */
    output        to_cpu_mem_req_ready,
        
    /** Cache responses to CPU: valid signal */
    output        to_cpu_cache_rsp_valid,
    /** Cache responses to CPU: 32-bit read data */
    output [31:0] to_cpu_cache_rsp_data,
    /** Acknowledgement from CPU: Ready to receive read data */
    input         from_cpu_cache_rsp_ready,
        
    //Memory/IO read interface
    /** Cache sending memory/IO read request: valid signal */
    output        to_mem_rd_req_valid,
    /** Cache sending memory read request: address
      * 4 byte alignment for I/O read 
      * 32 byte alignment for cache read miss */
    output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
      * 0 for I/O read (read only one data beat)
      * 7 for cache read miss (read eight data beats) */
    output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
    input	      from_mem_rd_req_ready,

    /** Memory return read data: valid signal of one data beat */
    input	      from_mem_rd_rsp_valid,
    /** Memory return read data: 32-bit one data beat */
    input  [31:0] from_mem_rd_rsp_data,
    /** Memory return read data: if current data beat is the last in this burst data transmission */
    input	      from_mem_rd_rsp_last,
    /** Acknowledgement from cache: ready to receive current data beat */
    output        to_mem_rd_rsp_ready,

    //Memory/IO write interface
    /** Cache sending memory/IO write request: valid signal */
    output        to_mem_wr_req_valid,
    /** Cache sending memory write request: address
      * 4 byte alignment for I/O write 
      * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
    output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
    output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
    input         from_mem_wr_req_ready,

    /** Cache sending memory/IO write data: valid signal for current data beat */
    output        to_mem_wr_data_valid,
    /** Cache sending memory/IO write data: current data beat */
    output [31:0] to_mem_wr_data,
    /** Cache sending memory/IO write data: write strobe
      * 4'b1111 for cache write-back 
      * other values for I/O write and cache write miss according to the original CPU request*/ 
    output [ 3:0] to_mem_wr_data_strb,
    /** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
    output        to_mem_wr_data_last,
    /** Acknowledgement from memory/IO: ready to receive current data beat */
    input	      from_mem_wr_data_ready
);

    // TODO: Please add your D-Cache code here
    wire [`ADDR_WIDTH - 1:0] set = from_cpu_mem_req_addr_reg[7:5];
    wire [23:0] tag = from_cpu_mem_req_addr_reg[31:8];
    wire [4:0] offset = from_cpu_mem_req_addr_reg[4:0];
    integer i;

    /********** valid array **********/
    reg [7:0] valid_array_way0;
    reg [7:0] valid_array_way1;
    reg [7:0] valid_array_way2;
    reg [7:0] valid_array_way3;

    always @(posedge clk) begin
        if (rst) begin
                valid_array_way0 <= 8'b0;
                valid_array_way1 <= 8'b0;
                valid_array_way2 <= 8'b0;
                valid_array_way3 <= 8'b0;
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

    always @(posedge clk) begin
        if (isNowREFILL) begin
            if (cache_hit_way0_reg) tag_array_way0[set] <= tag;
            if (cache_hit_way1_reg) tag_array_way1[set] <= tag;
            if (cache_hit_way2_reg) tag_array_way2[set] <= tag;
            if (cache_hit_way3_reg) tag_array_way3[set] <= tag;
        end
    end

    /********** data array(4 ways) **********/
    reg [255:0] data_array_way0 [7:0];
    reg [255:0] data_array_way1 [7:0];
    reg [255:0] data_array_way2 [7:0];
    reg [255:0] data_array_way3 [7:0];

    always @(posedge clk) begin
        if (isNowREFILL) begin
            if (cache_hit_way0_reg) data_array_way0[set] <= new_cache_line;
            if (cache_hit_way1_reg) data_array_way1[set] <= new_cache_line;
            if (cache_hit_way2_reg) data_array_way2[set] <= new_cache_line;
            if (cache_hit_way3_reg) data_array_way3[set] <= new_cache_line;
        end
        else if (isNowCACHE_WRITE) begin
            if (cache_hit_way0_reg) data_array_way0[set][{offset, 3'b0}+:32] <= from_cpu_mem_req_wdata_processed;
            if (cache_hit_way1_reg) data_array_way1[set][{offset, 3'b0}+:32] <= from_cpu_mem_req_wdata_processed;
            if (cache_hit_way2_reg) data_array_way2[set][{offset, 3'b0}+:32] <= from_cpu_mem_req_wdata_processed;
            if (cache_hit_way3_reg) data_array_way3[set][{offset, 3'b0}+:32] <= from_cpu_mem_req_wdata_processed;
        end
    end

    /********** dirty array **********/
    reg [`CACHE_WAY - 1:0] dirty_array [`CACHE_SET - 1:0];

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < `CACHE_SET; i = i + 1) begin
                dirty_array[i] <= 4'b0;
            end
        end
        else if (isNowCACHE_WRITE) begin
            if (cache_hit_way0_reg) dirty_array[set][0] <= 1'b1;
            if (cache_hit_way1_reg) dirty_array[set][1] <= 1'b1;
            if (cache_hit_way2_reg) dirty_array[set][2] <= 1'b1;
            if (cache_hit_way3_reg) dirty_array[set][3] <= 1'b1;
        end
        else if (isNowSEND) begin
            if (cache_hit_way0_reg) dirty_array[set][0] <= 1'b0;
            if (cache_hit_way1_reg) dirty_array[set][1] <= 1'b0;
            if (cache_hit_way2_reg) dirty_array[set][2] <= 1'b0;
            if (cache_hit_way3_reg) dirty_array[set][3] <= 1'b0;
        end
    end

    /********** Comparators and Cache-Hit-Signals **********/
    wire cache_hit_way0 = tag_array_way0[set] == tag & valid_array_way0[set];
    wire cache_hit_way1 = tag_array_way1[set] == tag & valid_array_way1[set];
    wire cache_hit_way2 = tag_array_way2[set] == tag & valid_array_way2[set];
    wire cache_hit_way3 = tag_array_way3[set] == tag & valid_array_way3[set];

    reg cache_hit_way0_reg;
    reg cache_hit_way1_reg;
    reg cache_hit_way2_reg;
    reg cache_hit_way3_reg;

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

    wire cache_hit  = cache_hit_way0 | cache_hit_way1 | cache_hit_way2 | cache_hit_way3;
    wire read_hit   = cache_hit & ~from_cpu_mem_req_reg;
    wire read_miss  = ~cache_hit & ~from_cpu_mem_req_reg;
    wire write_hit  = cache_hit & from_cpu_mem_req_reg;
    wire write_miss = ~cache_hit & from_cpu_mem_req_reg;

    wire all_valid = valid_array_way0[set] & valid_array_way1[set]
                   & valid_array_way2[set] & valid_array_way3[set];

    wire dirty = conflict_hit_way0 & valid_array_way0[set] & dirty_array[set][0]
               | conflict_hit_way1 & valid_array_way1[set] & dirty_array[set][1]
               | conflict_hit_way2 & valid_array_way2[set] & dirty_array[set][2]
               | conflict_hit_way3 & valid_array_way3[set] & dirty_array[set][3];

    /********** Bypass request signals **********/
    wire bypass = (~|from_cpu_mem_req_addr_reg[31:5])	// 0x00 - 0x1F
                | (|from_cpu_mem_req_addr_reg[31:30]);	// 0x40000000 - Inf
    wire bypass_read = bypass & ~from_cpu_mem_req_reg;
    wire bypass_write = bypass & from_cpu_mem_req_reg;

    /********** Read data from data array **********/
    wire [`LINE_LEN - 1:0] cache_data = {256{cache_hit_way0_reg}} & data_array_way0[set]
                                      | {256{cache_hit_way1_reg}} & data_array_way1[set]
                                      | {256{cache_hit_way2_reg}} & data_array_way2[set]
                                      | {256{cache_hit_way3_reg}} & data_array_way3[set];
    wire [31:0] cache_hit_data = cache_data[{offset, 3'b0}+:32];

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
    wire cache_hit_way01 = cache_hit_way0_reg | cache_hit_way1_reg;

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < `CACHE_SET; i = i + 1) begin
                PLRU_reg[i] <= 3'b0;
            end
        end
        if (isNowRESPONSE) begin
            PLRU_reg[set][0] <= cache_hit_way01;
            if (cache_hit_way01) PLRU_reg[set][1] <= cache_hit_way0_reg;
            if (~cache_hit_way01) PLRU_reg[set][2] <= cache_hit_way2_reg;
        end
    end

    /********** state machine **********/
    reg [14:0] current_state;
    reg [14:0] next_state;

    localparam CPU_WAIT     = 15'b000000000000001,
               TAG_READ     = 15'b000000000000010,
               CACHE_READ   = 15'b000000000000100,
               EVICT        = 15'b000000000001000,
               CACHE_WRITE  = 15'b000000000010000,
               MEM_READ     = 15'b000000000100000,
               MEM_WRITE    = 15'b000000001000000,
               RECEIVE      = 15'b000000010000000,
               SEND         = 15'b000000100000000,
               REFILL       = 15'b000001000000000,
               RESPONSE     = 15'b000010000000000,
               BY_READ_REQ  = 15'b000100000000000,
               BY_RECEIVE   = 15'b001000000000000,
               BY_WRITE_REQ = 15'b010000000000000,
               BY_SEND      = 15'b100000000000000;

    wire isNowCPU_WAIT     = current_state == CPU_WAIT;
    wire isNowTAG_READ     = current_state == TAG_READ;
    wire isNowCACHE_READ   = current_state == CACHE_READ;
    wire isNowEVICT        = current_state == EVICT;
    wire isNowCACHE_WRITE  = current_state == CACHE_WRITE;
    wire isNowMEM_READ     = current_state == MEM_READ;
    wire isNowMEM_WRITE    = current_state == MEM_WRITE;
    wire isNowRECEIVE      = current_state == RECEIVE;
    wire isNowSEND         = current_state == SEND;
    wire isNowREFILL       = current_state == REFILL;
    wire isNowRESPONSE     = current_state == RESPONSE;
    wire isNowBY_READ_REQ  = current_state == BY_READ_REQ;
    wire isNowBY_WRITE_REQ = current_state == BY_WRITE_REQ;
    wire isNowBY_RECEIVE   = current_state == BY_RECEIVE;
    wire isNowBY_SEND      = current_state == BY_SEND;

    always @(posedge clk) begin
        if (rst) current_state <= CPU_WAIT;
        else current_state <= next_state;
    end

    always @(*) begin
        case (current_state)
            CPU_WAIT: begin
                if (from_cpu_mem_req_valid) next_state = TAG_READ;
                else next_state = CPU_WAIT;
            end

            TAG_READ: begin
                if (read_hit & ~bypass) next_state = CACHE_READ;	    // read-hit
                else if ((read_miss | write_miss) & ~bypass) next_state = EVICT;	// read-miss or write-miss
                else if (write_hit & ~bypass) next_state = CACHE_WRITE;	// write-hit
                else if (bypass & ~from_cpu_mem_req_reg) next_state = BY_READ_REQ;
                else if (bypass & from_cpu_mem_req_reg) next_state = BY_WRITE_REQ;
                else next_state = TAG_READ;
            end

            CACHE_READ: next_state = RESPONSE;

            EVICT: begin
                if (dirty) next_state = MEM_WRITE;
                else next_state = MEM_READ;
            end

            MEM_READ: begin
                if (from_mem_rd_req_ready) next_state = RECEIVE;
                else next_state = MEM_READ;
            end

            RECEIVE: begin
                if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) next_state = REFILL;
                else next_state = RECEIVE;
            end

            REFILL: begin
                if (from_cpu_mem_req_reg) next_state = CACHE_WRITE;
                else next_state = RESPONSE;
            end

            RESPONSE: begin
                if (from_cpu_cache_rsp_ready) next_state = CPU_WAIT;
                else next_state = RESPONSE;
            end

            MEM_WRITE: begin
                if (from_mem_wr_req_ready) next_state = SEND;
                else next_state = MEM_WRITE;
            end

            SEND: begin
                if (from_mem_wr_data_ready & to_mem_wr_data_last) next_state = MEM_READ;
                else next_state = SEND;
            end

            CACHE_WRITE: next_state = CPU_WAIT;

            BY_READ_REQ: next_state = BY_RECEIVE;

            BY_WRITE_REQ: next_state = BY_SEND;

            BY_RECEIVE: begin
                if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) next_state = RESPONSE;
                else next_state = BY_RECEIVE;
            end

            BY_SEND: begin
                if (from_mem_wr_data_ready & to_mem_wr_data_last) next_state = CPU_WAIT;
                else next_state = BY_SEND;
            end
            default: next_state = CPU_WAIT;
        endcase
    end

    /********** deal with handshake signals **********/
    assign to_cpu_mem_req_ready   = isNowCPU_WAIT;
    assign to_cpu_cache_rsp_valid = isNowRESPONSE;

    assign to_mem_rd_req_valid    = isNowMEM_READ | isNowBY_READ_REQ;
    assign to_mem_rd_rsp_ready    = isNowRECEIVE  | isNowBY_RECEIVE | rst;

    assign to_mem_wr_req_valid    = isNowMEM_WRITE | isNowBY_WRITE_REQ;
    assign to_mem_wr_data_valid   = isNowSEND | isNowBY_SEND;
    
    /********** burst **********/
    reg [31:0] bypass_read_data;
    reg [`LINE_LEN - 1:0] new_cache_line;
    always @(posedge clk) begin
        if (isNowRECEIVE & from_mem_rd_rsp_valid) begin
            new_cache_line <= {from_mem_rd_rsp_data, new_cache_line[`LINE_LEN - 1:32]};
        end
    end

    always @(posedge clk) begin
        if (isNowBY_RECEIVE & from_mem_rd_rsp_valid) begin
            bypass_read_data <= from_mem_rd_rsp_data;
        end
    end

    /********** write to IO/mem signals **********/
    assign to_mem_wr_req_addr = {32{bypass}} & from_cpu_mem_req_addr_reg
                              | {32{~bypass & cache_hit_way0_reg}} & {tag_array_way0[set], set, 5'b0}
                              | {32{~bypass & cache_hit_way1_reg}} & {tag_array_way1[set], set, 5'b0}
                              | {32{~bypass & cache_hit_way2_reg}} & {tag_array_way2[set], set, 5'b0}
                              | {32{~bypass & cache_hit_way3_reg}} & {tag_array_way3[set], set, 5'b0};
    assign to_mem_wr_req_len = bypass ? 8'b0 : 8'h07;
    assign to_mem_wr_data_strb = bypass ? from_cpu_mem_req_wstrb_reg : 4'b1111;
    assign to_cpu_cache_rsp_data = bypass ? bypass_read_data : cache_hit_data;

    wire [31:0] from_cpu_mem_req_wdata_processed;
    assign from_cpu_mem_req_wdata_processed = {
        {8{from_cpu_mem_req_wstrb_reg[3]}} & from_cpu_mem_req_wdata_reg[31:24] | {8{~from_cpu_mem_req_wstrb_reg[3]}} & cache_hit_data[31:24],
        {8{from_cpu_mem_req_wstrb_reg[2]}} & from_cpu_mem_req_wdata_reg[23:16] | {8{~from_cpu_mem_req_wstrb_reg[2]}} & cache_hit_data[23:16],
        {8{from_cpu_mem_req_wstrb_reg[1]}} & from_cpu_mem_req_wdata_reg[15:8]  | {8{~from_cpu_mem_req_wstrb_reg[1]}} & cache_hit_data[15:8],
        {8{from_cpu_mem_req_wstrb_reg[0]}} & from_cpu_mem_req_wdata_reg[ 7:0]  | {8{~from_cpu_mem_req_wstrb_reg[0]}} & cache_hit_data[7:0]
    };

    // to_mem_data and to_mem_last 
    reg [`LINE_LEN - 1:0] write_data;
    reg [7:0] write_len;

    always @(posedge clk) begin
        if (isNowMEM_WRITE | isNowBY_WRITE_REQ) begin
            if (bypass) write_len <= 8'b00000001;
            else write_len <= 8'b10000000;
        end
        else if (isNowSEND && from_mem_wr_data_ready) write_len <= {1'b0, write_len[7:1]};
    end

    always @(posedge clk) begin
        if (isNowMEM_WRITE | isNowBY_WRITE_REQ) begin
            if (bypass) write_data <= {224'b0, from_cpu_mem_req_wdata_reg};
            else write_data <= cache_data;
        end
        else if (isNowSEND && from_mem_wr_data_ready) begin
            write_data <= {32'b0, write_data[255:32]};
        end
    end

    assign to_mem_wr_data = write_data[31:0];
    assign to_mem_wr_data_last = write_len[0];

    /********** store signals from cpu **********/
    reg from_cpu_mem_req_reg;
    reg [31:0] from_cpu_mem_req_addr_reg;
    reg [31:0] from_cpu_mem_req_wdata_reg;
    reg [3:0] from_cpu_mem_req_wstrb_reg;
    always @(posedge clk) begin
        if (isNowCPU_WAIT & from_cpu_mem_req_valid) begin
            from_cpu_mem_req_reg       <= from_cpu_mem_req;
            from_cpu_mem_req_addr_reg  <= from_cpu_mem_req_addr;
            from_cpu_mem_req_wdata_reg <= from_cpu_mem_req_wdata;
            from_cpu_mem_req_wstrb_reg <= from_cpu_mem_req_wstrb;
        end
    end

    assign to_mem_rd_req_len   = bypass ? 8'h0 : 8'h7;
    assign to_mem_rd_req_addr  = bypass ? from_cpu_mem_req_addr_reg : {from_cpu_mem_req_addr_reg[31:5], 5'b0};
endmodule
