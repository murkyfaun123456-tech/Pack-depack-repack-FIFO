`timescale 1ns/1ps

module pk_fifo_12in_32out #(
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
    reg [2:0] count;
    reg last [0:depth-1];
    reg winc;
    reg end_q;
    reg tail_q;
    reg [ADDR_WIDTH : 0] waddr, raddr;
    reg output_valid;
    reg [31:0] output_data;
    reg output_last;
    wire fifo_write, fifo_read;
    wire full, empty;
    wire [31:0] wdata;
    wire [31:0] rdata;
    assign din_hold = ~rstn || srst || full || end_q;
    // winc holds a completed word until the memory actually accepts it.
    assign fifo_write = rstn && ~srst && winc && ~full;
    assign fifo_read = rstn && ~srst && ~empty && ~dout_hold;

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            count <= 0;
        else if (srst)
            count <= 0;
        else if (din_vd && ~din_hold && din_last)
            count <= 0;
        else if (din_vd && ~din_hold)
            count <= count + 1;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer <= 0;
        else if (srst)
            buffer <= 0;
        // A split last needs a second, zero-padded write for the upper bits.
        else if (fifo_write && end_q && tail_q)
            buffer <= {32'd0, buffer[39:32]};
        else if (din_vd && ~din_hold)
            case (count)
                0 : buffer <= {28'd0, din};
                1 : buffer <= buffer | {16'd0, din, 12'd0};
                2 : buffer <= buffer | {4'd0, din, 24'd0};
                3 : buffer <= {24'd0, din, buffer[35:32]};
                4 : buffer <= buffer | {12'd0, din, 16'd0};
                5 : buffer <= buffer | {din, 28'd0};
                6 : buffer <= {20'd0, din, buffer[39:32]};
                7 : buffer <= buffer | {8'd0, din, 20'd0};
                default: buffer <= {24'd0, din};
            endcase
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
            din_last_reg <= din_last && count != 2 && count != 5;
        else if (fifo_write && end_q && tail_q)
            din_last_reg <= 1;
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
            winc <= 0;
        else if (srst)
            winc <= 0;
        else if (din_vd && ~din_hold)
            winc <= (count == 2 || count == 5 || count == 7) || din_last;
        else if (fifo_write && end_q && tail_q)
            winc <= 1;
        else if (fifo_write)
            winc <= 0;
    end

    // Block the next transaction until the last word (and any tail) is stored.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (din_vd && ~din_hold && din_last)
            end_q <= 1;
        else if (fifo_write && end_q && ~tail_q)
            end_q <= 0;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            tail_q <= 0;
        else if (srst)
            tail_q <= 0;
        else if (din_vd && ~din_hold && din_last)
            tail_q <= count == 2 || count == 5;
        else if (fifo_write && end_q && tail_q)
            tail_q <= 0;
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
    fifo_memory_V2 #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_fifo_memory (.clk(clk), .winc(fifo_write), .wdata(wdata), .rdata(rdata), .waddr(waddr[ADDR_WIDTH-1:0]), .raddr(raddr[ADDR_WIDTH-1:0]));

endmodule



module fifo_memory #(
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
