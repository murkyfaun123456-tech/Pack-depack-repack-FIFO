`timescale 1ns/1ps
// Unpack 32-bit words, least-significant bits first, into 12-bit samples.
// At din_last, discard the final incomplete sample (0 to 11 padding bits).
module dpk_fifo_32in_12out_V3 #(
    parameter integer FIFO_ADDR_WIDTH = 4 // >= 2; depth = 2**FIFO_ADDR_WIDTH
) (
    output reg [11:0] dout,
    output reg        dout_last,
    output reg        dout_vd,
    output wire       din_hold,
    input  wire       rstn,
    input  wire       clk,
    input  wire       srst,
    input  wire [31:0] din,
    input  wire       din_last,
    input  wire       din_vd,
    input  wire       dout_hold
);

    // Register declarations
    reg [63:0] bits_q;
    reg [6:0] count_q;
    reg end_q;

    // Wire declarations
    wire [32:0] fifo_data;
    wire fifo_full;
    wire fifo_empty;
    wire fifo_overflow;
    wire fifo_pop;
    wire output_take;
    wire output_last;

    // Input FIFO control
    assign fifo_pop = rstn && !srst && !fifo_empty && !end_q && (count_q <= 7'd32);
    assign din_hold = !rstn || srst || fifo_full;

    dpk_v3_async_fifo #(.DATA_WIDTH(33), .ADDR_WIDTH(FIFO_ADDR_WIDTH)) input_fifo (
        .wclk(clk), .wrst_n(rstn), .wsrst(srst),
        .winc(din_vd && !din_hold), .wdata({din_last, din}),
        .wfull(fifo_full), .wovr(fifo_overflow),
        .rclk(clk), .rrst_n(rstn), .rsrst(srst),
        .rinc(fifo_pop), .rdata(fifo_data), .rempty(fifo_empty)
    );

    // Output data: hold preserves the registered word.
    assign output_take = !dout_hold && (count_q >= 7'd12);

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            dout <= 0;
        else if (srst)
            dout <= 0;
        else if (output_take)
            dout <= bits_q[11:0];
    end

    // Last travels with the registered data and is held with it.
    assign output_last = end_q && (count_q < 7'd24);

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            dout_last <= 0;
        else if (srst)
            dout_last <= 0;
        else if (!dout_hold)
            dout_last <= output_take && output_last;
    end

    // Output valid.
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            dout_vd <= 0;
        else if (srst)
            dout_vd <= 0;
        else if (!dout_hold)
            dout_vd <= output_take;
    end

    // Bit count: handle simultaneous output removal and input insertion.
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            count_q <= 0;
        else if (srst)
            count_q <= 0;
        else if (output_take && output_last)
            count_q <= 0;
        else if (output_take && fifo_pop)
            count_q <= count_q - 7'd12 + 7'd32;
        else if (output_take)
            count_q <= count_q - 7'd12;
        else if (fifo_pop)
            count_q <= count_q + 7'd32;
    end

    // Data: shift old bits first, then append new bits at the freed position.
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            bits_q <= 0;
        else if (srst)
            bits_q <= 0;
        else if (output_take && output_last)
            bits_q <= 0;
        else if (output_take && fifo_pop)
            bits_q <= (bits_q >> 12) | ({32'b0, fifo_data[31:0]} << (count_q - 7'd12));
        else if (output_take)
            bits_q <= bits_q >> 12;
        else if (fifo_pop)
            bits_q <= bits_q | ({32'b0, fifo_data[31:0]} << count_q);
    end

    // Reading stops at a pending end, so a final output cannot coincide
    // with a new input transfer into this bit buffer.
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (fifo_pop && fifo_data[32])
            end_q <= 1;
        else if (output_take && output_last)
            end_q <= 0;
    end

endmodule

// FIFO implementation copied from async_fifo.v with unique module names.
// V3 change: synchronous reset also clears both Gray-pointer synchronizers.
module dpk_v3_async_fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4
) (
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire                  wsrst,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire                  wfull,

    input  wire                   rclk,
    input  wire                   rrst_n,
    input  wire                   rinc,
    input  wire                   rsrst,
    output wire  [DATA_WIDTH-1:0] rdata,
    output wire                   rempty,
    output wire                   wovr
);

    // Wire declarations
    wire [ADDR_WIDTH-1 : 0] waddr;
    wire [ADDR_WIDTH-1 : 0] raddr;
    wire [ADDR_WIDTH : 0] wptr;
    wire [ADDR_WIDTH : 0] rptr;
    wire [ADDR_WIDTH : 0] rq2_wptr;
    wire [ADDR_WIDTH : 0] wq2_rptr;

    dpk_v3_fifomem #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) mem (
        .wclk(wclk), .winc(winc), .waddr(waddr), .wdata(wdata), .wfull(wfull),
        .raddr(raddr), .rdata(rdata)
    );
    dpk_v3_sync #(.WIDTH(ADDR_WIDTH+1)) sync_w2r (
        .clk(rclk), .rst_n(rrst_n), .srst(rsrst), .gray_in(wptr), .gray_out(rq2_wptr)
    );
    dpk_v3_sync #(.WIDTH(ADDR_WIDTH+1)) sync_r2w (
        .clk(wclk), .rst_n(wrst_n), .srst(wsrst), .gray_in(rptr), .gray_out(wq2_rptr)
    );

    dpk_v3_wptr_full #(.ADDR_WIDTH(ADDR_WIDTH)) dpk_v3_wptr_full (
        .wclk(wclk), .winc(winc), .wrst_n(wrst_n), .wq2_rptr(wq2_rptr),.wsrst(wsrst), .wptr(wptr), .waddr(waddr), .wfull(wfull), .wovr(wovr)
    );
    dpk_v3_rptr_empty #(.ADDR_WIDTH(ADDR_WIDTH)) dpk_v3_rptr_empty (
        .rclk(rclk), .rinc(rinc), .rrst_n(rrst_n), .rq2_wptr(rq2_wptr),.rsrst(rsrst), .rptr(rptr), .raddr(raddr), .rempty(rempty)
    );
endmodule

module dpk_v3_fifomem # (parameter DATA_WIDTH = 8,
                  parameter ADDR_WIDTH = 4) 
   (input                     winc, wfull, wclk,
    input   [DATA_WIDTH-1 : 0]  wdata,
    input   [ADDR_WIDTH-1 : 0]  waddr, raddr,     
    output  [DATA_WIDTH-1 : 0]  rdata);

    // Constants
    localparam DEPTH = 1<<ADDR_WIDTH;

    // Register declarations
    reg [DATA_WIDTH-1 : 0] mem [0 : DEPTH-1];

    // Continuous assignments
    assign rdata = mem[raddr];

    always @(posedge wclk)
        if (winc && ~wfull)
            mem[waddr] <= wdata;

endmodule

module dpk_v3_sync #(parameter WIDTH = 5) (
    input wire clk, rst_n, srst,
    input wire [WIDTH-1:0] gray_in,
    output wire [WIDTH-1:0] gray_out
);

    // Register declarations
    reg [WIDTH-1:0] stage1;
    reg [WIDTH-1:0] stage2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stage1 <= 0;
        else if (srst)
            stage1 <= 0;
        else
            stage1 <= gray_in;
    end
    assign gray_out = stage2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stage2 <= 0;
        else if (srst)
            stage2 <= 0;
        else
            stage2 <= stage1;
    end

endmodule

module dpk_v3_wptr_full # (parameter ADDR_WIDTH = 4) 
   (output                          wfull,
    output      [ADDR_WIDTH-1 : 0]  waddr,
    output reg  [ADDR_WIDTH : 0]    wptr, 
    output reg                         wovr,
    input       [ADDR_WIDTH : 0]    wq2_rptr, 
    input                           winc, wclk, wrst_n, wsrst);

    // Register declarations
    reg [ADDR_WIDTH : 0] addr;

    // Wire declarations
    wire [ADDR_WIDTH : 0] next_gray;
    wire [ADDR_WIDTH : 0] next_addr;

    assign waddr = addr[ADDR_WIDTH-1:0];
    assign next_addr = addr + (winc & ~wfull);

    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) 
            addr <= 0;
        else if (wsrst)
            addr <= 0;
        else if (winc & ~wfull)         
            addr <= next_addr;

    assign next_gray = (next_addr >> 1) ^ next_addr;

    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) 
            wptr <= 0;
        else if (wsrst)
            wptr <= 0;
        else if (winc & ~wfull)         
            wptr <= next_gray;

    assign wfull = (wptr == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1], wq2_rptr[ADDR_WIDTH-2:0]});

    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) 
            wovr <= 0;
        else if (wsrst)
            wovr <= 0;
        else if (winc & wfull)         
            wovr <= 1;

endmodule

module dpk_v3_rptr_empty # (parameter ADDR_WIDTH = 4) 
   (output                          rempty,
    output      [ADDR_WIDTH-1 : 0]  raddr,
    output reg  [ADDR_WIDTH : 0]    rptr, 
    input       [ADDR_WIDTH : 0]    rq2_wptr, 
    input                           rinc, rclk, rrst_n, rsrst);

    // Register declarations
    reg [ADDR_WIDTH : 0] addr;

    // Wire declarations
    wire [ADDR_WIDTH : 0] next_gray;
    wire [ADDR_WIDTH : 0] next_addr;

    assign raddr = addr[ADDR_WIDTH-1:0];
    assign next_addr = addr + (rinc & ~rempty);

    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) 
            addr <= 0;
        else if (rsrst)
            addr <= 0;
        else if ((rinc & ~rempty))          
            addr <= next_addr;

    assign next_gray = (next_addr >> 1) ^ next_addr;
    assign rempty = (rptr == rq2_wptr);

    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) 
            rptr <= 0;
        else if (rsrst)
            rptr <= 0;
        else if ((rinc & ~rempty))       
            rptr <= next_gray;

endmodule
