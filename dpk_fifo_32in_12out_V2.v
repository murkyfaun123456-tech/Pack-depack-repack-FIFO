`timescale 1ns/1ps

module dpk_fifo_32in_12out_V2 #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 3
) (
    output wire [11:0] dout,
    output wire        dout_last,
    output wire        dout_vd,
    output wire        din_hold,
    input wire         rstn,
    input wire         clk,
    input wire         srst,
    input wire [31:0]  din,
    input wire         din_last,
    input wire         din_vd,
    input wire         dout_hold

);
    localparam depth = 1 << ADDR_WIDTH;
    integer i;
    reg [31:0] buffer_in;
    reg [39:0] buffer;
    reg [5:0] count;
    reg last [0:depth-1];
    reg winc;
    reg end_q;
    reg [ADDR_WIDTH : 0] waddr, raddr;
    reg output_valid;
    reg [11:0] output_data;
    reg output_last;
    wire fifo_write, fifo_read;
    wire output_take, output_end;
    wire [5:0] remaining_count;
    wire [39:0] remaining_buffer;
    wire [39:0] input_bits;
    wire full, empty;
    wire [31:0] wdata;
    wire [31:0] rdata;

    assign din_hold = ~rstn || srst || full;
    assign fifo_write = rstn && ~srst && winc && ~full;
    // Remove the outgoing 12 bits before appending a new 32-bit word.
    assign output_take = rstn && ~srst && ~dout_hold && (count >= 12);
    assign output_end = output_take && end_q && (count < 24);
    assign remaining_count = output_take ? count - 6'd12 : count;
    assign remaining_buffer = output_take ? buffer >> 12 : buffer;
    assign fifo_read = rstn && ~srst && ~empty && ~end_q && (remaining_count <= 8);
    // When fifo_read=1, the insertion offset can only be 0, 4, or 8 bits.
    assign input_bits = (remaining_count == 0) ? {8'd0, rdata} : (remaining_count == 4) ? {4'd0, rdata, 4'd0} : {rdata, 8'd0};

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            count <= 0;
        else if (srst)
            count <= 0;
        else if (output_end)
            count <= 0;
        else if (fifo_read || output_take)
            count <= remaining_count + (fifo_read ? 6'd32 : 6'd0);
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer_in <= 0;
        else if (srst)
            buffer_in <= 0;
        else if (din_vd && ~din_hold)
            buffer_in <= din;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer <= 0;
        else if (srst)
            buffer <= 0;
        // Discard the incomplete 4/8-bit tail at a transaction boundary.
        else if (output_end)
            buffer <= 0;
        else if (fifo_read || output_take)
            buffer <= remaining_buffer | (fifo_read ? input_bits : 40'd0);
    end

    reg din_last_reg;
    wire [1:0] last_state;
    assign last_state = {fifo_write, fifo_read};
    assign dout_last = output_last;

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            din_last_reg <= 0;
        else if (srst)
            din_last_reg <= 0;
        else if (din_vd && ~din_hold)
            din_last_reg <= din_last;
        else if (fifo_write)
            din_last_reg <= 0;
    end


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
                2'b10: last[waddr[ADDR_WIDTH-1:0]] <= din_last_reg;
                2'b11: begin
                    last[raddr[ADDR_WIDTH-1:0]] <= 0;
                    last[waddr[ADDR_WIDTH-1:0]] <= din_last_reg;
                end
                default: begin
                    last[raddr[ADDR_WIDTH-1:0]] <= last[raddr[ADDR_WIDTH-1:0]];
                    last[waddr[ADDR_WIDTH-1:0]] <= last[waddr[ADDR_WIDTH-1:0]];
                end
            endcase
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_data <= 0;
        else if (srst)
            output_data <= 0;
        else if (output_take)
            output_data <= buffer[11:0];
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            winc <= 0;
        else if (srst)
            winc <= 0;
        else if (din_vd && ~din_hold)
            winc <= 1;
        else if (fifo_write)
            winc <= 0;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            waddr <= 0;
        else if (srst)
            waddr <= 0;
        else if (fifo_write)
            waddr <= waddr + 1;
    end

    assign wdata = buffer_in;


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

    // Stop reading across last until its final complete segment is emitted.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (output_end)
            end_q <= 0;
        else if (fifo_read)
            end_q <= last[raddr[ADDR_WIDTH-1:0]];
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
    dpk_fifo_memory_V2 #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_fifo_memory (.clk(clk), .winc(fifo_write), .wdata(wdata), .rdata(rdata), .waddr(waddr[ADDR_WIDTH-1:0]), .raddr(raddr[ADDR_WIDTH-1:0]));

endmodule



module dpk_fifo_memory_V2 #(
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
