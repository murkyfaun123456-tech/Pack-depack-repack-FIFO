`timescale 1ns/1ps

module dpk_fifo_32in_12out_V1 #(
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
    reg [31:0] buffer_0, buffer_1;
    reg [2:0] count;
    reg last [0:depth-1];
    reg winc;
    reg end_q;
    reg word_ready;
    reg [ADDR_WIDTH : 0] waddr, raddr;
    reg output_valid;
    reg [11:0] output_data;
    reg output_last;
    wire fifo_write, fifo_read;
    wire output_take, word_done;
    wire full, empty;
    wire [31:0] wdata;
    wire [31:0] rdata;

    assign din_hold = ~rstn || srst || full;
    assign fifo_write = rstn && ~srst && winc && ~full;
    // Load a word before phases 0, 2, and 5; keep its remainder for crossings.
    assign fifo_read = rstn && ~srst && ~empty && ~word_ready;
    assign output_take = rstn && ~srst && word_ready && ~dout_hold;
    assign word_done = count == 1 || count == 4 || count == 7;

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            count <= 0;
        else if (srst)
            count <= 0;
        else if (output_take && end_q && word_done)
            count <= 0;
        else if (output_take)
            count <= count + 1;
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
            buffer_0 <= 0;
        else if (srst)
            buffer_0 <= 0;
        else if (fifo_read && (count == 0 || count == 5))
            buffer_0 <= rdata;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            buffer_1 <= 0;
        else if (srst)
            buffer_1 <= 0;
        else if (fifo_read && count == 2)
            buffer_1 <= rdata;
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
            case (count)
                0 : output_data <= buffer_0[11:0];
                1 : output_data <= buffer_0[23:12];
                2 : output_data <= {buffer_1[3:0], buffer_0[31:24]};
                3 : output_data <= buffer_1[15:4];
                4 : output_data <= buffer_1[27:16];
                5 : output_data <= {buffer_0[7:0], buffer_1[31:28]};
                6 : output_data <= buffer_0[19:8];
                7 : output_data <= buffer_0[31:20];
                default: output_data <= 0;
            endcase
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
            output_valid <= word_ready;
    end

    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            output_last <= 0;
        else if (srst)
            output_last <= 0;
        else if (~dout_hold)
            output_last <= word_ready && end_q && word_done;
    end

    // A loaded word supplies 2, 3, or 3 outputs at phases 0..1, 2..4, 5..7.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            word_ready <= 0;
        else if (srst)
            word_ready <= 0;
        else if (fifo_read)
            word_ready <= 1;
        else if (output_take && word_done)
            word_ready <= 0;
    end

    // Save the input word's last until its final complete 12-bit segment.
    always @(posedge clk or negedge rstn) begin
        if (~rstn)
            end_q <= 0;
        else if (srst)
            end_q <= 0;
        else if (fifo_read)
            end_q <= last[raddr[ADDR_WIDTH-1:0]];
        else if (output_take && word_done)
            end_q <= 0;
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
    dpk_fifo_memory_V1 #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) u_fifo_memory (.clk(clk), .winc(fifo_write), .wdata(wdata), .rdata(rdata), .waddr(waddr[ADDR_WIDTH-1:0]), .raddr(raddr[ADDR_WIDTH-1:0]));

endmodule



module dpk_fifo_memory_V1 #(
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
