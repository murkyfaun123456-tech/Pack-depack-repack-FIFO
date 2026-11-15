`timescale 1ns/1ps
// Adjacent-line pairing: (line 0, line 1), (line 1, line 2), ...
// Both line stores use the supplied async FIFO architecture, on one clock.
// When a current-line word is popped, it is also pushed back into the same
// FIFO, preserving the line for the next pair. The old previous line drains.
// Require 1 <= din_len <= min(4095, 2**FIFO_ADDR_WIDTH - 1).
// Leave one FIFO entry free so simultaneous pop/write-back can start.
module repack_blk_V3 #(
    parameter integer FIFO_ADDR_WIDTH = 12 // >= 2; depth of EACH line FIFO
) (
    output wire [23:0] dout,
    output wire        dout_last,
    output wire        dout_vd,
    output wire        din_hold,
    input  wire        rstn,
    input  wire        clk,
    input  wire        srst,
    input  wire [31:0] din,
    input  wire        din_last,
    input  wire        din_vd,
    input  wire        dout_hold,
    input  wire [11:0] din_len
);

    // Constants
    localparam [1:0] LOAD_FIRST = 2'd0;
    localparam [1:0] LOAD_NEXT = 2'd1;
    localparam [1:0] OUTPUT_PAIR = 2'd2;

    // Register declarations
    reg [1:0] state;
    reg previous_bank;
    reg [11:0] write_count;
    reg [11:0] read_count;
    reg [11:0] line_words;
    reg read_finished;

    // Wire declarations
    wire [32:0] fifo0_data;
    wire [32:0] fifo1_data;
    wire full0;
    wire full1;
    wire empty0;
    wire empty1;
    wire overflow0;
    wire overflow1;
    wire previous_hold;
    wire current_hold;
    wire [11:0] previous_sample;
    wire [11:0] current_sample;
    wire previous_vd;
    wire current_vd;
    wire previous_last;
    wire current_last;
    wire load_bank;
    wire load_full;
    wire current_full;
    wire active;
    wire input_push;
    wire pair_pop;
    wire push0;
    wire push1;
    wire [32:0] data0_in;
    wire [32:0] data1_in;
    wire [32:0] previous_word;
    wire [32:0] current_word;
    wire pair_done;
    wire start_pair;

    // Line FIFO routing and input handshake

    assign load_bank = (state == LOAD_FIRST) ? 1'b0 : !previous_bank;
    assign load_full = load_bank ? full1 : full0;
    assign current_full = previous_bank ? full0 : full1;
    assign active = rstn && !srst;
    assign din_hold = !active || (state == OUTPUT_PAIR) || load_full || (write_count == 12'd4095);
    assign input_push = din_vd && !din_hold;
    assign pair_pop = active && (state == OUTPUT_PAIR) && !read_finished && !empty0 && !empty1 && !previous_hold && !current_hold && !current_full;
    assign push0 = (input_push && !load_bank) || (pair_pop && previous_bank);
    assign push1 = (input_push && load_bank) || (pair_pop && !previous_bank);
    assign data0_in = (state == OUTPUT_PAIR) ? fifo0_data : {din_last, din};
    assign data1_in = (state == OUTPUT_PAIR) ? fifo1_data : {din_last, din};

    // FIFO rdata is the current head (asynchronous read in fifomem).
    // Feed BOTH unpackers and advance BOTH read pointers on this one event.

    repack_v3_async_fifo #(.DATA_WIDTH(33), .ADDR_WIDTH(FIFO_ADDR_WIDTH)) line_fifo0 (
        .wclk(clk), .wrst_n(rstn), .wsrst(srst), .winc(push0),
        .wdata(data0_in), .wfull(full0), .wovr(overflow0),
        .rclk(clk), .rrst_n(rstn), .rsrst(srst), .rinc(pair_pop),
        .rdata(fifo0_data), .rempty(empty0)
    );
    repack_v3_async_fifo #(.DATA_WIDTH(33), .ADDR_WIDTH(FIFO_ADDR_WIDTH)) line_fifo1 (
        .wclk(clk), .wrst_n(rstn), .wsrst(srst), .winc(push1),
        .wdata(data1_in), .wfull(full1), .wovr(overflow1),
        .rclk(clk), .rrst_n(rstn), .rsrst(srst), .rinc(pair_pop),
        .rdata(fifo1_data), .rempty(empty1)
    );

    assign previous_word = previous_bank ? fifo1_data : fifo0_data;
    assign current_word = previous_bank ? fifo0_data : fifo1_data;

    repack_v3_unpack_32in_12out unpack_previous (
        .clk(clk), .rstn(rstn), .srst(srst),
        .din(previous_word[31:0]), .din_last(previous_word[32]),
        .din_vd(pair_pop), .din_hold(previous_hold),
        .dout(previous_sample), .dout_last(previous_last), .dout_vd(previous_vd),
        .dout_hold(dout_hold)
    );
    repack_v3_unpack_32in_12out unpack_current (
        .clk(clk), .rstn(rstn), .srst(srst),
        .din(current_word[31:0]), .din_last(current_word[32]),
        .din_vd(pair_pop), .din_hold(current_hold),
        .dout(current_sample), .dout_last(current_last), .dout_vd(current_vd),
        .dout_hold(dout_hold)
    );

    assign dout = {current_sample, previous_sample};
    assign dout_vd = previous_vd && current_vd;
    assign dout_last = previous_last && current_last;
    assign pair_done = (state == OUTPUT_PAIR) && dout_vd && !dout_hold && dout_last;

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            state <= LOAD_FIRST;
        else if (srst)
            state <= LOAD_FIRST;
        else begin
            if (input_push && din_last) begin
                if (state == LOAD_FIRST)
                    state <= LOAD_NEXT;
                else
                    state <= OUTPUT_PAIR;
            end
            if (pair_done)
                state <= LOAD_NEXT;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            previous_bank <= 0;
        else if (srst)
            previous_bank <= 0;
        else begin
            if (pair_done)
                previous_bank <= !previous_bank;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            write_count <= 0;
        else if (srst)
            write_count <= 0;
        else begin
            if (input_push) begin
                if (din_last)
                    write_count <= 0;
                else
                    write_count <= write_count + 12'd1;
            end
            if (pair_done)
                write_count <= 0;
        end
    end

    assign start_pair = input_push && din_last && (state == LOAD_NEXT);

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            read_count <= 0;
        else if (srst)
            read_count <= 0;
        else begin
            if (start_pair)
                read_count <= 0;
            if (pair_pop && (read_count != line_words - 12'd1))
                read_count <= read_count + 12'd1;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            line_words <= 0;
        else if (srst)
            line_words <= 0;
        else begin
            if (input_push && (write_count == 0))
                line_words <= din_len;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            read_finished <= 0;
        else if (srst)
            read_finished <= 0;
        else begin
            if (start_pair)
                read_finished <= 0;
            if (pair_pop && (read_count == line_words - 12'd1))
                read_finished <= 1;
        end
    end

endmodule

// Local 64-bit unpacking buffer. Only complete 12-bit samples are emitted.
module repack_v3_unpack_32in_12out (
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
    wire input_take;
    wire output_take;
    wire output_last;

    // Input transfer control
    assign din_hold = end_q || (count_q > 7'd32);

    assign input_take = din_vd && !din_hold;

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
        else if (output_take && input_take)
            count_q <= count_q - 7'd12 + 7'd32;
        else if (output_take)
            count_q <= count_q - 7'd12;
        else if (input_take)
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
        else if (output_take && input_take)
            bits_q <= (bits_q >> 12) | ({32'b0, din} << (count_q - 7'd12));
        else if (output_take)
            bits_q <= bits_q >> 12;
        else if (input_take)
            bits_q <= bits_q | ({32'b0, din} << count_q);
    end

    // Reading stops at a pending end, so a final output cannot coincide
    // with a new input transfer into this bit buffer.
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (input_take && din_last)
            end_q <= 1;
        else if (output_take && output_last)
            end_q <= 0;
    end

endmodule

// FIFO implementation copied from async_fifo.v with unique module names.
// V3 change: synchronous reset also clears both Gray-pointer synchronizers.
module repack_v3_async_fifo #(
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

    repack_v3_fifomem #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) mem (
        .wclk(wclk), .winc(winc), .waddr(waddr), .wdata(wdata), .wfull(wfull),
        .raddr(raddr), .rdata(rdata)
    );
    repack_v3_sync #(.WIDTH(ADDR_WIDTH+1)) sync_w2r (
        .clk(rclk), .rst_n(rrst_n), .srst(rsrst), .gray_in(wptr), .gray_out(rq2_wptr)
    );
    repack_v3_sync #(.WIDTH(ADDR_WIDTH+1)) sync_r2w (
        .clk(wclk), .rst_n(wrst_n), .srst(wsrst), .gray_in(rptr), .gray_out(wq2_rptr)
    );

    repack_v3_wptr_full #(.ADDR_WIDTH(ADDR_WIDTH)) repack_v3_wptr_full (
        .wclk(wclk), .winc(winc), .wrst_n(wrst_n), .wq2_rptr(wq2_rptr),.wsrst(wsrst), .wptr(wptr), .waddr(waddr), .wfull(wfull), .wovr(wovr)
    );
    repack_v3_rptr_empty #(.ADDR_WIDTH(ADDR_WIDTH)) repack_v3_rptr_empty (
        .rclk(rclk), .rinc(rinc), .rrst_n(rrst_n), .rq2_wptr(rq2_wptr),.rsrst(rsrst), .rptr(rptr), .raddr(raddr), .rempty(rempty)
    );
endmodule

module repack_v3_fifomem # (parameter DATA_WIDTH = 8,
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

module repack_v3_sync #(parameter WIDTH = 5) (
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

module repack_v3_wptr_full # (parameter ADDR_WIDTH = 4) 
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

module repack_v3_rptr_empty # (parameter ADDR_WIDTH = 4) 
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
