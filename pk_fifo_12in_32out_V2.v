`timescale 1ns/1ps

module pk_fifo_12in_32out_V2 #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 3
) (
    output wire [31:0] dout,
    output wire        dout_last,
    output wire        dout_vd,
    output wire        din_hold,
    input wire         rstn,
    input wire         clk,
    input wire         srst,
    input wire [11:0]  din,
    input wire         din_last,
    input wire         din_vd,
    input wire         dout_hold

);
    localparam depth = 1 << ADDR_WIDTH;
    integer i;
    reg [39:0] buffer;
    reg [5:0] count;
    reg last [0:depth-1];
    reg end_q;
    reg [ADDR_WIDTH : 0] waddr, raddr;
    reg output_valid;
    reg [31:0] output_data;
    reg output_last;
    wire fifo_write, fifo_read;
    wire input_take, word_last;
    wire [5:0] remaining_count;
    wire [39:0] remaining_buffer;
    wire [39:0] input_bits;
    wire full, empty;
    wire [31:0] wdata;
    wire [31:0] rdata;
    // Remove a complete/padded word before appending an accepted input.
    assign fifo_write = rstn && ~srst && ~full && ((count >= 32) || (end_q && count != 0));
    assign fifo_read = rstn && ~srst && ~empty && ~dout_hold;
    assign word_last = end_q && (count <= 32);
    assign remaining_count = fifo_write ? ((count > 32) ? count - 6'd32 : 6'd0) : count;
    assign remaining_buffer = fifo_write ? buffer >> 32 : buffer;
    assign din_hold = ~rstn || srst || end_q || (remaining_count > 28);
    assign input_take = din_vd && ~din_hold;
    // Legal input positions are multiples of four from 0 through 28.
    assign input_bits = {28'd0, din} << {remaining_count[4:2], 2'b00};

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            count <= 0;
        else if (srst)
            count <= 0;
        else if (fifo_write || input_take)
            count <= remaining_count + (input_take ? 6'd12 : 6'd0);
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer <= 0;
        else if (srst)
            buffer <= 0;
        else if (fifo_write || input_take)
            buffer <= remaining_buffer | (input_take ? input_bits : 40'd0);
    end

    wire [1:0] last_state;
    assign last_state = {fifo_write, fifo_read};
    assign dout_last = output_last;

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            for (i = 0; i < depth; i = i + 1)
                last[i] <= 0;
        else if (srst)
            for (i = 0; i < depth; i = i + 1)
                last[i] <= 0;
        else if (fifo_write || fifo_read) begin
            case (last_state)
                2'b01: last[raddr[ADDR_WIDTH-1:0]] <= 0;
                2'b10: last[waddr[ADDR_WIDTH-1:0]] <= word_last;
                2'b11: begin
                    last[raddr[ADDR_WIDTH-1:0]] <= 0;
                    last[waddr[ADDR_WIDTH-1:0]] <= word_last;
                end
                default: begin
                    last[raddr[ADDR_WIDTH-1:0]] <= last[raddr[ADDR_WIDTH-1:0]];
                    last[waddr[ADDR_WIDTH-1:0]] <= last[waddr[ADDR_WIDTH-1:0]];
                end
            endcase
        end
    end

    // Stop the next transaction until every final bit is stored, including
    // a second zero-padded word when the ending leaves 36 or 40 valid bits.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (input_take && din_last)
            end_q <= 1;
        else if (fifo_write && word_last)
            end_q <= 0;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            waddr <= 0;
        else if (srst)
            waddr <= 0;
        else if (fifo_write)
            waddr <= waddr + 1;
    end

    assign wdata = buffer[31:0];


    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_valid <= 0;
        else if (srst)
            output_valid <= 0;
        else if (~dout_hold)
            output_valid <= ~empty;
    end

    // Keep all output fields stable while held, including an invalid output.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_data <= 0;
        else if (srst)
            output_data <= 0;
        else if (fifo_read)
            output_data <= rdata;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_last <= 0;
        else if (srst)
            output_last <= 0;
        else if (~dout_hold)
            output_last <= ~empty && last[raddr[ADDR_WIDTH-1:0]];
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            raddr <= 0;
        else if (srst)
            raddr <= 0;
        else if (fifo_read)
            raddr <= raddr + 1;
    end

    assign full = (~waddr[ADDR_WIDTH] == raddr[ADDR_WIDTH]) && (waddr[ADDR_WIDTH-1:0] == raddr[ADDR_WIDTH-1:0]);
    assign empty = waddr == raddr;
    assign dout_vd = output_valid;
    assign dout = output_data;
    pk_fifo_memory_V2 #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_fifo_memory (.clk(clk), .winc(fifo_write), .wdata(wdata), .rdata(rdata), .waddr(waddr[ADDR_WIDTH-1:0]), .raddr(raddr[ADDR_WIDTH-1:0]));

endmodule



module pk_fifo_memory_V2 #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 3
) (
    input       clk,
    input       winc,
    input       [DATA_WIDTH-1 : 0] wdata,
    output      [DATA_WIDTH-1 : 0] rdata,
    input       [ADDR_WIDTH-1 : 0] waddr,
    input       [ADDR_WIDTH-1 : 0] raddr
);
    localparam depth = 1 << ADDR_WIDTH;
    reg [DATA_WIDTH-1 : 0] mem [0 : depth - 1];

    always @(posedge clk) begin
        if (winc)
            mem[waddr] <= wdata;
    end
    assign rdata = mem[raddr];

endmodule
