`timescale 1ns/1ps

module tb_dpk_fifo_32in_12out_V2;
    parameter integer FIFO_ADDR_WIDTH = 3;
    parameter TEST_PPT = 1;
    parameter TEST_CONTINUOUS = 1;
    parameter TEST_RANDOM = 1;
    parameter TEST_SRST = 0;
    parameter integer CONTINUOUS_WORDS = 512;
    parameter integer RANDOM_PACKETS = 64;
    parameter integer SEED = 1;
    parameter DUMP_WAVE = 1;

    reg clk = 0;
    always #5 clk = ~clk;
    reg rstn, srst;
    reg [31:0] din;
    reg din_last, din_vd, dout_hold;
    wire [11:0] dout;
    wire dout_last, dout_vd, din_hold;

    localparam READ_NORMAL = 0, READ_RANDOM = 1, READ_PPT = 2, READ_HOLD = 3;
    integer read_mode = READ_NORMAL, hold_count = 0;
    integer write_seed, read_seed;
    reg model_enable = 1;
    reg [11:0] expected [0:65535];
    reg expected_last [0:65535];
    reg [11:0] model_word = 0;
    integer model_bits = 0, expected_count = 0, received = 0;
    integer accepted = 0, last_count = 0, tests_passed = 0;
    integer bit_index;
    reg input_pending = 0, output_pending = 0;
    reg [33:0] saved_input;
    reg [13:0] saved_output;

    dpk_fifo_32in_12out_V2 #(.ADDR_WIDTH(FIFO_ADDR_WIDTH)) dut (
        .rstn(rstn), .clk(clk), .srst(srst),
        .din(din), .din_last(din_last), .din_vd(din_vd),
        .din_hold(din_hold), .dout(dout), .dout_last(dout_last),
        .dout_vd(dout_vd), .dout_hold(dout_hold)
    );

    // One call writes one item. Valid/data/last stay unchanged under hold.
    task send;
        input [31:0] data;
        input last;
        begin
            @(negedge clk);
            din = data; din_last = last; din_vd = 1;
            @(posedge clk);
            while (din_hold) @(posedge clk);
        end
    endtask

    task idle;
        input integer cycles;
        begin
            if (cycles > 0) begin
                @(negedge clk);
                din_vd = 0; din_last = 0;
                repeat (cycles - 1) @(negedge clk);
            end
        end
    endtask

    task drain;
        begin
            idle(1);
            wait (received == expected_count);
            repeat (12) @(negedge clk);
            if (model_bits != 0 || received != expected_count || dout_vd !== 0 || dout_last !== 0)
                $fatal(1, "Drain failed or extra output detected");
        end
    endtask

    task finish_test;
        input [8*16-1:0] name;
        begin
            drain;
            tests_passed = tests_passed + 1;
            $display("PASS %0s: total_in=%0d total_out=%0d total_last=%0d", name, accepted, received, last_count);
        end
    endtask

    task pulse_srst;
        begin
            @(negedge clk);
            din_vd = 0; din_last = 0; srst = 1;
            #1;
            if (din_hold !== 1) $fatal(1, "Input not held during srst");
            repeat (2) @(negedge clk);
            srst = 0;
        end
    endtask

    task test_ppt;
        integer item;
        begin
            read_mode = READ_PPT;
            model_enable = 0; // Use the explicit PPT expected values below.
            expected_count = 12;
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
            for (item = 0; item < 12; item = item + 1)
                expected_last[item] = (item == 7) || (item == 9) || (item == 11);
            send(32'h56123001, 0); // D02
            send(32'hfabc7894, 0); // D25
            send(32'h246135de, 1); // D57, last
            send(32'h00468357, 1); // D89, last
            send(32'h0068a579, 1); // DAB, last
            finish_test("PPT");
            model_enable = 1;
        end
    endtask

    task test_continuous;
        integer packet, item;
        begin
            read_mode = READ_NORMAL;
            // No idle between writes. Short packets cover the ending phases.
            for (packet = 1; packet <= 8; packet = packet + 1)
                for (item = 0; item < packet; item = item + 1)
                    send(packet * 37 + item, item == packet - 1);
            // One transaction much longer than the FIFO capacity.
            for (item = 0; item < CONTINUOUS_WORDS; item = item + 1)
                send(32'h12345678 + item * 32'h10203041, item == CONTINUOUS_WORDS - 1);
            finish_test("CONTINUOUS");
        end
    endtask

    task test_random;
        integer packet, item, length;
        begin
            read_mode = READ_RANDOM;
            for (packet = 0; packet < RANDOM_PACKETS; packet = packet + 1) begin
                length = ($random(write_seed) & 31) + 1;
                for (item = 0; item < length; item = item + 1) begin
                    send($random(write_seed), item == length - 1);
                    idle($random(write_seed) & 3);
                end
            end
            finish_test("RANDOM");
        end
    endtask

    task test_srst;
        begin
            // Reset incomplete/queued data, then verify a fresh transaction.
            read_mode = READ_HOLD;
            send(32'h89abcdef, 0);
            send(32'h76543210, 0);
            idle(4);
            pulse_srst;
            read_mode = READ_NORMAL;
            send(32'hff123456, 1);
            drain;

            // Reset while a valid last output is held.
            send(32'hff135246, 1);
            idle(1);
            wait (dout_vd && dout_last);
            @(negedge clk);
            #1; read_mode = READ_HOLD; dout_hold = 1;
            repeat (3) @(negedge clk);
            if (!dout_vd || !dout_last) $fatal(1, "No held last before srst");
            pulse_srst;
            read_mode = READ_NORMAL;
            send(32'hff789abc, 1);
            finish_test("SRST");
        end
    endtask

    // Main stimulus: enable each test independently with its parameter.
    initial begin
        if (DUMP_WAVE) begin
            $dumpfile("tb_dpk_fifo_32in_12out_V2.vcd");
            $dumpvars(0, tb_dpk_fifo_32in_12out_V2);
        end
        rstn = 0; srst = 0;
        din = 0; din_last = 0; din_vd = 0; dout_hold = 0;
        write_seed = SEED;
        read_seed = SEED ^ 32'h13579bdf;
        repeat (3) @(negedge clk);
        rstn = 1;

        if (TEST_PPT)        test_ppt;
        if (TEST_CONTINUOUS) test_continuous;
        if (TEST_RANDOM)     test_random;
        if (TEST_SRST)       test_srst;

        if (tests_passed == 0)
            $display("SKIP: all tests disabled");
        else
            $display("PASS DePack V2: %0d tests, depth=%0d seed=%0d", tests_passed, 1 << FIFO_ADDR_WIDTH, SEED);
        $finish;
    end

    // The read side only changes hold on falling edges.
    always @(negedge clk) begin
        if (!rstn || srst) begin
            dout_hold = 0; hold_count = 0;
        end else begin
            case (read_mode)
                READ_NORMAL: dout_hold = 0;
                READ_RANDOM: dout_hold = (($random(read_seed) & 3) != 0);
                READ_PPT: begin
                    if (!dout_vd || hold_count == 3) begin
                        dout_hold = 0; hold_count = 0;
                    end else begin
                        dout_hold = 1; hold_count = hold_count + 1;
                    end
                end
                READ_HOLD: dout_hold = 1;
            endcase
            if (read_mode != READ_PPT) hold_count = 0;
        end
    end

    // Compare public handshakes only; no dependency on DUT internal signals.
    always @(posedge clk) begin
        if (!rstn || srst) begin
            model_word = 0; model_bits = 0;
            expected_count = 0; received = 0;
            accepted = 0; last_count = 0;
            input_pending = 0; output_pending = 0;
            #1;
            if ({dout_vd, dout_last, dout} !== 14'd0)
                $fatal(1, "Output did not reset");
        end else begin
            if (input_pending && {din_vd, din_last, din} !== saved_input)
                $fatal(1, "Input changed before acceptance");
            if (output_pending && {dout_vd, dout_last, dout} !== saved_output)
                $fatal(1, "Output changed before hold release");
            input_pending = din_vd && din_hold;
            output_pending = dout_hold;
            saved_input = {din_vd, din_last, din};
            saved_output = {dout_vd, dout_last, dout};
            if (dout_vd !== 0 && dout_vd !== 1)
                $fatal(1, "Output valid is unknown");

            if (din_vd && !din_hold) begin
                accepted = accepted + 1;
                if (model_enable) begin
                    for (bit_index = 0; bit_index < 32; bit_index = bit_index + 1) begin
                        model_word[model_bits] = din[bit_index];
                        model_bits = model_bits + 1;
                        if (model_bits == 12) begin
                            if (expected_count >= 65536) $fatal(1, "Expected queue full");
                            expected[expected_count] = model_word;
                            expected_last[expected_count] = 0;
                            expected_count = expected_count + 1;
                            model_word = 0; model_bits = 0;
                        end
                    end
                    if (din_last) begin
                        expected_last[expected_count-1] = 1;
                        model_word = 0; model_bits = 0;
                    end
                end
            end
            if (dout_vd && !dout_hold) begin
                if (received >= expected_count)
                    $fatal(1, "Unexpected output data=%h last=%b", dout, dout_last);
                if (dout !== expected[received] || dout_last !== expected_last[received])
                    $fatal(1, "Output %0d got %h/%b expected %h/%b", received, dout, dout_last, expected[received], expected_last[received]);
                received = received + 1;
                if (dout_last) last_count = last_count + 1;
            end
            #1;
            if (output_pending && {dout_vd, dout_last, dout} !== saved_output)
                $fatal(1, "Output changed during hold");
        end
    end

    initial begin
        #2000000;
        $fatal(1, "DePack V2 timeout");
    end
endmodule
