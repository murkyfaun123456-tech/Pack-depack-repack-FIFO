`timescale 1ns/1ps

module repack_blk_V2 #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 12
) (
    output wire [23:0] dout,
    output wire        dout_last,
    output wire        dout_vd,
    output wire        din_hold,
    input wire         rstn,
    input wire         clk,
    input wire         srst,
    input wire [31:0]  din,
    input wire [11:0]  din_len,
    input wire         din_last,
    input wire         din_vd,
    input wire         dout_hold

);
    localparam depth = 1 << ADDR_WIDTH;
    localparam LOAD_FIRST = 2'd0, REPACK_LINE = 2'd1, FINISH_LINE = 2'd2;
    reg [1:0] state;
    reg bank_sel;
    reg [11:0] line_len;
    reg [ADDR_WIDTH-1:0] waddr;
    reg [31:0] buffer_in;
    reg din_last_reg;
    reg pair_valid;
    reg [39:0] buffer_0, buffer_1;
    reg [5:0] count;
    reg end_q;
    reg [23:0] output_data;
    reg output_valid;
    reg output_last;
    wire [11:0] input_len;
    wire input_take, line_end, line_done;
    wire write0, write1, read0, read1;
    wire [31:0] rdata0, rdata1;
    wire [31:0] previous_word;
    wire pair_take, output_take, output_end;
    wire [5:0] remaining_count;
    wire [39:0] remaining_0, remaining_1;
    wire [39:0] input_0, input_1;

    // bank_sel selects the previous line; the other bank receives this line.
    assign input_len = (waddr == 0) ? din_len : line_len;
    assign din_hold = ~rstn || srst || (state == FINISH_LINE) || (input_len == 0) || (input_len > depth) || ((state != LOAD_FIRST) && (waddr == 0) && (din_len != line_len)) || ((state == REPACK_LINE) && pair_valid && ~pair_take);
    assign input_take = din_vd && ~din_hold;
    assign line_end = input_take && din_last;
    assign line_done = rstn && ~srst && output_valid && output_last && ~dout_hold;
    assign write0 = input_take && ((state == LOAD_FIRST) || bank_sel);
    assign write1 = input_take && (state == REPACK_LINE) && ~bank_sel;
    assign read0 = input_take && (state == REPACK_LINE) && ~bank_sel;
    assign read1 = input_take && (state == REPACK_LINE) && bank_sel;
    assign previous_word = bank_sel ? rdata1 : rdata0;

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            state <= LOAD_FIRST;
        else if (srst)
            state <= LOAD_FIRST;
        else
            case (state)
                LOAD_FIRST: if (line_end) state <= REPACK_LINE;
                REPACK_LINE: if (line_end) state <= FINISH_LINE;
                FINISH_LINE: if (line_done) state <= REPACK_LINE;
                default: state <= LOAD_FIRST;
            endcase
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            bank_sel <= 0;
        else if (srst)
            bank_sel <= 0;
        else if (line_done)
            bank_sel <= ~bank_sel;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            line_len <= 0;
        else if (srst)
            line_len <= 0;
        else if (input_take && waddr == 0)
            line_len <= din_len;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            waddr <= 0;
        else if (srst)
            waddr <= 0;
        else if (line_end)
            waddr <= 0;
        else if (input_take)
            waddr <= waddr + 1'b1;
    end

    // The current word and last wait alongside the synchronous RAM response.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer_in <= 0;
        else if (srst)
            buffer_in <= 0;
        else if (input_take && state == REPACK_LINE)
            buffer_in <= din;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            din_last_reg <= 0;
        else if (srst)
            din_last_reg <= 0;
        else if (input_take && state == REPACK_LINE)
            din_last_reg <= din_last;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            pair_valid <= 0;
        else if (srst)
            pair_valid <= 0;
        else if (input_take && state == REPACK_LINE)
            pair_valid <= 1;
        else if (pair_take)
            pair_valid <= 0;
    end

    // Both lines advance together, sharing the valid bit count and last.
    assign output_take = rstn && ~srst && ~dout_hold && (count >= 12);
    assign output_end = output_take && end_q && (count < 24);
    assign remaining_count = output_take ? count - 6'd12 : count;
    assign remaining_0 = output_take ? buffer_0 >> 12 : buffer_0;
    assign remaining_1 = output_take ? buffer_1 >> 12 : buffer_1;
    assign pair_take = rstn && ~srst && pair_valid && ~end_q && (remaining_count <= 8);
    assign input_0 = (remaining_count == 0) ? {8'd0, previous_word} : (remaining_count == 4) ? {4'd0, previous_word, 4'd0} : {previous_word, 8'd0};
    assign input_1 = (remaining_count == 0) ? {8'd0, buffer_in} : (remaining_count == 4) ? {4'd0, buffer_in, 4'd0} : {buffer_in, 8'd0};

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            count <= 0;
        else if (srst)
            count <= 0;
        else if (output_end)
            count <= 0;
        else if (pair_take || output_take)
            count <= remaining_count + (pair_take ? 6'd32 : 6'd0);
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer_0 <= 0;
        else if (srst)
            buffer_0 <= 0;
        else if (output_end)
            buffer_0 <= 0;
        else if (pair_take || output_take)
            buffer_0 <= remaining_0 | (pair_take ? input_0 : 40'd0);
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer_1 <= 0;
        else if (srst)
            buffer_1 <= 0;
        else if (output_end)
            buffer_1 <= 0;
        else if (pair_take || output_take)
            buffer_1 <= remaining_1 | (pair_take ? input_1 : 40'd0);
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (output_end)
            end_q <= 0;
        else if (pair_take)
            end_q <= din_last_reg;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_data <= 0;
        else if (srst)
            output_data <= 0;
        else if (output_take)
            output_data <= {buffer_1[11:0], buffer_0[11:0]};
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_valid <= 0;
        else if (srst)
            output_valid <= 0;
        else if (~dout_hold)
            output_valid <= count >= 12;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_last <= 0;
        else if (srst)
            output_last <= 0;
        else if (~dout_hold)
            output_last <= output_end;
    end

    assign dout = output_data;
    assign dout_vd = output_valid;
    assign dout_last = output_last;

    repack_line_memory_V2 #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_line0 (.clk(clk), .winc(write0), .rinc(read0), .addr(waddr), .wdata(din), .rdata(rdata0));
    repack_line_memory_V2 #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_line1 (.clk(clk), .winc(write1), .rinc(read1), .addr(waddr), .wdata(din), .rdata(rdata1));

endmodule


module repack_line_memory_V2 #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 12
) (
    input       clk,
    input       winc,
    input       rinc,
    input       [ADDR_WIDTH-1:0] addr,
    input       [DATA_WIDTH-1:0] wdata,
    output reg  [DATA_WIDTH-1:0] rdata
);
    localparam depth = 1 << ADDR_WIDTH;
    reg [DATA_WIDTH-1:0] mem [0:depth-1];

    always @(posedge clk) begin
        if (winc)
            mem[addr] <= wdata;
    end

    always @(posedge clk) begin
        if (rinc)
            rdata <= mem[addr];
    end

endmodule
