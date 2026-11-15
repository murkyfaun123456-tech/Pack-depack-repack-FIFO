`timescale 1ns/1ps
module tb_pk_fifo_12in_32out_V3;
    parameter integer FIFO_ADDR_WIDTH = 4;
    // Short single-word transactions fill the FIFO while output is held.
    localparam STRESS_INPUTS = (1 << FIFO_ADDR_WIDTH) + 8;
    localparam TOTAL_OUTPUTS = 5 + STRESS_INPUTS;
    reg clk = 0;
    always #5 clk = ~clk;

    reg rstn, srst;
    reg [11:0] din;
    reg din_last, din_vd, dout_hold;
    wire [31:0] dout;
    wire dout_last, dout_vd, din_hold;
    reg [31:0] expected [0:TOTAL_OUTPUTS-1];
    reg expected_last [0:TOTAL_OUTPUTS-1];
    integer received = 0;
    integer i;
    integer accepted = 0, last_count = 0;
    integer input_hold_cycles = 0, input_last_hold_cycles = 0;
    integer output_hold_cycles = 0, output_last_hold_cycles = 0;
    reg input_pending = 0, output_pending = 0;
    reg [13:0] saved_input;
    reg [33:0] saved_output;
    reg fill_fifo = 0;


    pk_fifo_12in_32out_V3 #(.FIFO_ADDR_WIDTH(FIFO_ADDR_WIDTH)) dut (
        .clk(clk), .rstn(rstn), .srst(srst),
        .din(din), .din_last(din_last), .din_vd(din_vd),
        .din_hold(din_hold), .dout(dout), .dout_last(dout_last),
        .dout_vd(dout_vd), .dout_hold(dout_hold)
    );

    // One input per call; keep data and last unchanged if the DUT stalls.
    task send;
        input [11:0] data;
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
        $dumpfile("tb_pk_fifo_12in_32out_V3.vcd");
        $dumpvars(0, tb_pk_fifo_12in_32out_V3);
        rstn = 0; srst = 0;
        din = 0; din_last = 0; din_vd = 0; dout_hold = 0;
        expected[0] = 32'h56123001; expected_last[0] = 0; // D02
        expected[1] = 32'hfabc7894; expected_last[1] = 0; // D25
        expected[2] = 32'h246135de; expected_last[2] = 1; // D57
        expected[3] = 32'h00468357; expected_last[3] = 1; // D89, pad 8 MSBs
        expected[4] = 32'h00000579; expected_last[4] = 1; // DA, pad 20 MSBs
        for (i = 0; i < STRESS_INPUTS; i = i + 1) begin
            expected[5+i] = i;
            expected_last[5+i] = 1;
        end
        repeat (2) @(negedge clk);
        rstn = 1;

        // PPT page 1: D0..D7, D8..D9, then D10, in three transactions.
        send(12'h001, 0); // D0
        send(12'h123, 0); // D1
        send(12'h456, 0); // D2
        send(12'h789, 0); // D3
        send(12'habc, 0); // D4
        send(12'hdef, 0); // D5
        send(12'h135, 0); // D6
        send(12'h246, 1); // D7, last
        send(12'h357, 0); // D8
        send(12'h468, 1); // D9, last
        send(12'h579, 1); // D10, last
        @(negedge clk);
        din_vd = 0; din_last = 0;

        wait (received == 5);
        repeat (4) @(negedge clk);
        // Hold output until the input FIFO fills. All these inputs have
        // last=1, so this explicitly tests last held at the input boundary.
        @(negedge clk); #1; fill_fifo = 1;
        for (i = 0; i < STRESS_INPUTS; i = i + 1)
            send(i[11:0], 1);
        @(negedge clk); din_vd = 0; din_last = 0;
        wait (received == TOTAL_OUTPUTS);
        // Allow delayed FIFO state and any unwanted extra outputs to surface.
        repeat (20) @(negedge clk);
        if (accepted != 11 + STRESS_INPUTS || last_count != 3 + STRESS_INPUTS)
            $fatal(1, "Pack: incorrect accepted input or output last count");
        if (input_last_hold_cycles == 0 || output_last_hold_cycles == 0)
            $fatal(1, "Pack: last+hold scenario was not exercised");
        $display("PASS: V3 Pack: in=%0d out=%0d last=%0d, input_hold=%0d (last=%0d), output_hold=%0d (last=%0d)",
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
            if (received >= TOTAL_OUTPUTS) $fatal(1, "Extra Pack output");
            if (dout !== expected[received] || dout_last !== expected_last[received])
                $fatal(1, "Pack output %0d: data=%h last=%b", received, dout, dout_last);
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
        $fatal(1, "Pack timeout");
    end
endmodule
