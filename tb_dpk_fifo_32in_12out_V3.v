`timescale 1ns/1ps
module tb_dpk_fifo_32in_12out_V3;
    parameter integer FIFO_ADDR_WIDTH = 4;
    // Short single-word transactions fill the FIFO while output is held.
    localparam STRESS_INPUTS = (1 << FIFO_ADDR_WIDTH) + 8;
    localparam TOTAL_OUTPUTS = 12 + 2 * STRESS_INPUTS;
    reg clk = 0;
    always #5 clk = ~clk;

    reg rstn, srst;
    reg [31:0] din;
    reg din_last, din_vd, dout_hold;
    wire [11:0] dout;
    wire dout_last, dout_vd, din_hold;
    reg [11:0] expected [0:TOTAL_OUTPUTS-1];
    reg expected_last [0:TOTAL_OUTPUTS-1];
    integer received = 0;
    integer i;
    integer accepted = 0, last_count = 0;
    integer input_hold_cycles = 0, input_last_hold_cycles = 0;
    integer output_hold_cycles = 0, output_last_hold_cycles = 0;
    reg input_pending = 0, output_pending = 0;
    reg [33:0] saved_input;
    reg [13:0] saved_output;
    reg fill_fifo = 0;


    dpk_fifo_32in_12out_V3 #(.FIFO_ADDR_WIDTH(FIFO_ADDR_WIDTH)) dut (
        .clk(clk), .rstn(rstn), .srst(srst),
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
        $dumpfile("tb_dpk_fifo_32in_12out_V3.vcd");
        $dumpvars(0, tb_dpk_fifo_32in_12out_V3);
        rstn = 0; srst = 0;
        din = 0; din_last = 0; din_vd = 0; dout_hold = 0;
        expected[0] = 12'h001; // D0
        expected[1] = 12'h123; // D1
        expected[2] = 12'h456; // D2
        expected[3] = 12'h789; // D3
        expected[4] = 12'habc; // D4
        expected[5] = 12'hdef; // D5
        expected[6] = 12'h135; // D6
        expected[7] = 12'h246; // D7, last
        expected[8] = 12'h357; // D8
        expected[9] = 12'h468; // D9, last
        expected[10] = 12'h579; // DA
        expected[11] = 12'h68a; // DB, last
        for (i = 0; i < 12; i = i + 1)
            expected_last[i] = (i == 7) || (i == 9) || (i == 11);
        for (i = 0; i < STRESS_INPUTS; i = i + 1) begin
            expected[12+2*i] = i;
            expected[13+2*i] = i + 1;
            expected_last[12+2*i] = 0;
            expected_last[13+2*i] = 1;
        end
        repeat (2) @(negedge clk);
        rstn = 1;

        // PPT page 3: 3 words, 1 word, 1 word, in three transactions.
        send(32'h56123001, 0); // D02
        send(32'hfabc7894, 0); // D25
        send(32'h246135de, 1); // D57, last: 8 samples in total
        send(32'h00468357, 1); // D89, last: ignore 8 padding bits
        send(32'h0068a579, 1); // DAB, last: start at a new boundary
        @(negedge clk);
        din_vd = 0; din_last = 0;

        wait (received == 12);
        repeat (4) @(negedge clk);
        // Hold output until the input FIFO fills. All these inputs have
        // last=1, so this explicitly tests last held at the input boundary.
        @(negedge clk); #1; fill_fifo = 1;
        for (i = 0; i < STRESS_INPUTS; i = i + 1)
            send(((i+1) << 12) | i, 1);
        @(negedge clk); din_vd = 0; din_last = 0;
        wait (received == TOTAL_OUTPUTS);
        // Allow delayed FIFO state and any unwanted extra outputs to surface.
        repeat (20) @(negedge clk);
        if (accepted != 5 + STRESS_INPUTS || last_count != 3 + STRESS_INPUTS)
            $fatal(1, "DePack: incorrect accepted input or output last count");
        if (input_last_hold_cycles == 0 || output_last_hold_cycles == 0)
            $fatal(1, "DePack: last+hold scenario was not exercised");
        $display("PASS: V3 DePack: in=%0d out=%0d last=%0d, input_hold=%0d (last=%0d), output_hold=%0d (last=%0d)",
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
            if (fill_fifo) begin
                dout_hold = 1;
                wait (din_vd && din_hold);
                repeat (3) @(negedge clk);
                dout_hold = 0;
                fill_fifo = 0;
                @(posedge clk);
            end else if (dout_vd) begin
                dout_hold = 1;
                repeat (3) @(negedge clk);
                dout_hold = 0;
                @(posedge clk);
            end
        end
    end

    always @(posedge clk) begin
        if (rstn && !srst && dout_vd && !dout_hold) begin
            if (received >= TOTAL_OUTPUTS) $fatal(1, "Extra DePack output");
            if (dout !== expected[received] ||
                dout_last !== expected_last[received])
                $fatal(1, "DePack output %0d: data=%h last=%b", received, dout, dout_last);
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
            if ((dut.fifo_pop && dut.fifo_empty) || dut.fifo_overflow)
                $fatal(1, "FIFO empty read / overflow");
            // Observe registered outputs after nonblocking updates as well.
            #1;
            if (output_pending && {dout_vd, dout_last, dout} !== saved_output)
                $fatal(1, "Output valid/data/last changed during hold");
        end
    end

    initial begin
        #200000;
        $fatal(1, "DePack timeout");
    end
endmodule
