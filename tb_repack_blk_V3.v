`timescale 1ns/1ps
module tb_repack_blk_V3;
    parameter integer FIFO_ADDR_WIDTH = 12;
    localparam TOTAL_OUTPUTS = 14;
    reg clk = 0;
    always #5 clk = ~clk;

    reg rstn, srst;
    reg [31:0] din;
    reg [11:0] din_len;
    reg din_last, din_vd, dout_hold;
    wire [23:0] dout;
    wire dout_last, dout_vd, din_hold;
    reg [23:0] expected [0:TOTAL_OUTPUTS-1];
    reg expected_last [0:TOTAL_OUTPUTS-1];
    integer received = 0;
    integer i;
    integer accepted = 0, last_count = 0;
    integer input_hold_cycles = 0, input_last_hold_cycles = 0;
    integer output_hold_cycles = 0, output_last_hold_cycles = 0;
    reg input_pending = 0, output_pending = 0;
    reg [33:0] saved_input;
    reg [25:0] saved_output;
    reg fill_fifo = 0;


    repack_blk_V3 #(.FIFO_ADDR_WIDTH(FIFO_ADDR_WIDTH)) dut (
        .clk(clk), .rstn(rstn), .srst(srst), .din_len(din_len),
        .din(din), .din_last(din_last), .din_vd(din_vd),
        .din_hold(din_hold), .dout(dout), .dout_last(dout_last),
        .dout_vd(dout_vd), .dout_hold(dout_hold)
    );

    task send;
        input [31:0] data;
        input last;
        begin
            @(negedge clk);
            din_vd = 1;
            din = data;
            din_last = last;
            @(posedge clk);
            while (din_hold) @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("tb_repack_blk_V3.vcd");
        $dumpvars(0, tb_repack_blk_V3);
        rstn = 0; srst = 0;
        din = 0; din_last = 0; din_vd = 0; dout_hold = 0;
        din_len = 12'd2;
        expected[0] = 24'hdef001; // {D5, D0}
        expected[1] = 24'h135123; // {D6, D1}
        expected[2] = 24'h246456; // {D7, D2}
        expected[3] = 24'h357789; // {D8, D3}
        expected[4] = 24'h468abc; // {D9, D4}, last
        expected[5] = 24'h579def; // {DA, D5}
        expected[6] = 24'h68a135; // {DB, D6}
        expected[7] = 24'h79b246; // {DC, D7}
        expected[8] = 24'h8ac357; // {DD, D8}
        expected[9] = 24'h9bd468; // {DE, D9}, last
        expected[10] = 24'h003001;
        expected[11] = 24'h004002;
        expected[12] = 24'h005003;
        expected[13] = 24'h006004;
        for (i = 0; i < TOTAL_OUTPUTS; i = i + 1)
            expected_last[i] = (i == 4) || (i == 9) || (i == 11) || (i == 13);
        repeat (2) @(negedge clk);
        rstn = 1;

        // PPT page 6: two words per line, five valid samples per line.
        send(32'h56123001, 0); // Previous line: D02
        send(32'h0abc7894, 1); // Previous line: D24, last
        send(32'h46135def, 0); // Current line:  D57
        send(32'h04683572, 1); // Current line:  D79, last
        // A third line verifies that the current line was kept in its FIFO.
        // send waits for din_hold while the first pair is being output.
        send(32'h9b68a579, 0); // Next line: DAC
        send(32'h09bd8ac7, 1); // Next line: DCE, last
        @(negedge clk);
        din_vd = 0; din_last = 0;

        wait (received == 10);
        repeat (4) @(negedge clk);
        // Restart with one-word lines: the third input has last=1 while
        // din_hold is high during output of the first line pair.
        @(negedge clk); srst = 1;
        @(negedge clk); srst = 0; din_len = 1;
        send(32'h00002001, 1);
        send(32'h00004003, 1);
        send(32'h00006005, 1);
        @(negedge clk); din_vd = 0; din_last = 0;
        wait (received == TOTAL_OUTPUTS);
        // Allow delayed FIFO state and any unwanted extra outputs to surface.
        repeat (20) @(negedge clk);
        if (accepted != 9 || last_count != 4)
            $fatal(1, "RePack: incorrect accepted input or output last count");
        if (input_last_hold_cycles == 0 || output_last_hold_cycles == 0)
            $fatal(1, "RePack: last+hold scenario was not exercised");
        $display("PASS: V3 RePack: in=%0d out=%0d last=%0d, input_hold=%0d (last=%0d), output_hold=%0d (last=%0d)",
                 accepted, received, last_count, input_hold_cycles,
                 input_last_hold_cycles, output_hold_cycles, output_last_hold_cycles);
        $finish;
    end

    // Pause EVERY output, including every last word, for three clocks.
    // Drive hold on falling edges so the receiving edge is unambiguous.
    initial begin
        wait (rstn);
        forever begin
            @(negedge clk);
            if (dout_vd) begin
                dout_hold = 1;
                repeat (3) @(negedge clk);
                dout_hold = 0;
                @(posedge clk);
            end
        end
    end

    always @(posedge clk) begin
        if (rstn && !srst && dout_vd && !dout_hold) begin
            if (received >= TOTAL_OUTPUTS) $fatal(1, "Extra RePack output");
            if (dout !== expected[received] || dout_last !== expected_last[received])
                $fatal(1, "RePack output %0d: data=%h last=%b", received, dout, dout_last);
            if (dout_last) last_count = last_count + 1;
            received = received + 1;
        end
    end

    // Check both sides at every clock, including the hold-release edge.
    always @(posedge clk) begin
        if (!rstn || srst) begin
            input_pending = 0;
            output_pending = 0;
        end else begin
            if (input_pending && {din_vd, din_last, din} !== saved_input)
                $fatal(1, "Input valid/data/last changed before acceptance");
            if (output_pending && {dout_vd, dout_last, dout} !== saved_output)
                $fatal(1, "Output valid/data/last changed before hold release");
            input_pending = din_vd && din_hold;
            output_pending = dout_hold;
            saved_input = {din_vd, din_last, din};
            saved_output = {dout_vd, dout_last, dout};
            if (din_vd && !din_hold) accepted = accepted + 1;
            if (din_vd && din_hold) begin
                input_hold_cycles = input_hold_cycles + 1;
                if (din_last) input_last_hold_cycles = input_last_hold_cycles + 1;
            end
            if (dout_vd && dout_hold) begin
                output_hold_cycles = output_hold_cycles + 1;
                if (dout_last) output_last_hold_cycles = output_last_hold_cycles + 1;
            end
            if ((dut.pair_pop && (dut.empty0 || dut.empty1)) ||
                (dut.push0 && dut.full0) || (dut.push1 && dut.full1) ||
                dut.overflow0 || dut.overflow1)
                $fatal(1, "Line FIFO empty read / full write / overflow");
            // Observe registered outputs after nonblocking updates as well.
            #1;
            if (output_pending && {dout_vd, dout_last, dout} !== saved_output)
                $fatal(1, "Output valid/data/last changed during hold");
        end
    end

    initial begin
        #200000;
        $fatal(1, "RePack timeout");
    end
endmodule
