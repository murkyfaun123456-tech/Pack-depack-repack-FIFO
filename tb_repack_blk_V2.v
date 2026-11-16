`timescale 1ns/1ps

module tb_repack_blk_V2;
    parameter integer FIFO_ADDR_WIDTH = 12;
    parameter TEST_PPT = 1;
    parameter TEST_CONTINUOUS = 1;
    parameter TEST_RANDOM = 1;
    parameter TEST_SRST = 0;
    parameter integer CONTINUOUS_WORDS = 512;
    parameter integer CONTINUOUS_LINES = 4;
    parameter integer RANDOM_LINES = 64;
    parameter integer SEED = 1;
    parameter DUMP_WAVE = 1;

    reg clk = 0;
    always #5 clk = ~clk;
    reg rstn, srst;
    reg [31:0] din;
    reg [11:0] din_len;
    reg din_last, din_vd, dout_hold;
    wire [23:0] dout;
    wire dout_last, dout_vd, din_hold;

    localparam READ_NORMAL = 0, READ_RANDOM = 1, READ_PPT = 2, READ_HOLD = 3;
    integer read_mode = READ_NORMAL, hold_count = 0;
    integer write_seed, read_seed;
    reg model_enable = 1;
    localparam MAX_WORDS = ((1 << FIFO_ADDR_WIDTH) < 4095) ? (1 << FIFO_ADDR_WIDTH) : 4095;
    localparam LONG_WORDS = (CONTINUOUS_WORDS < MAX_WORDS) ? CONTINUOUS_WORDS : MAX_WORDS;
    localparam RANDOM_LIMIT = (MAX_WORDS < 32) ? MAX_WORDS : 32;
    reg [23:0] expected [0:65535];
    reg [11:0] previous_line [0:10919];
    reg previous_valid = 0;
    integer sample_index = 0, write_index = 0, model_len = 0;
    reg expected_last [0:65535];
    reg [11:0] model_word = 0;
    integer model_bits = 0, expected_count = 0, received = 0;
    integer accepted = 0, last_count = 0, tests_passed = 0;
    integer bit_index;
    reg input_pending = 0, output_pending = 0;
    reg [45:0] saved_input;
    reg [25:0] saved_output;

    repack_blk_V2 #(.ADDR_WIDTH(FIFO_ADDR_WIDTH)) dut (
        .rstn(rstn), .clk(clk), .srst(srst), .din_len(din_len),
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
            if (model_bits != 0 || write_index != 0 || received != expected_count || dout_vd !== 0 || dout_last !== 0)
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

    task reset_dut;
        begin
            @(negedge clk);
            rstn = 0; srst = 0; din_vd = 0; din_last = 0;
            read_mode = READ_NORMAL; model_enable = 1;
            repeat (3) @(negedge clk);
            rstn = 1;
        end
    endtask

    task send_line;
        input [31:0] data;
        integer item;
        begin
            for (item = 0; item < din_len; item = item + 1)
                send(data ^ (item * 32'h10203041), item == din_len - 1);
        end
    endtask

    task test_ppt;
        integer item;
        begin
            reset_dut;
            din_len = 2;
            read_mode = READ_PPT;
            model_enable = 0;
            expected_count = 10;
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
            for (item = 0; item < 10; item = item + 1)
                expected_last[item] = (item == 4) || (item == 9);

            send(32'h56123001, 0); // L1: D02
            send(32'h0abc7894, 1); // L1: D24, last
            send(32'h46135def, 0); // L2: D57
            send(32'h04683572, 1); // L2: D79, last
            send(32'h9b68a579, 0); // L3: DAC
            send(32'h09bd8ac7, 1); // L3: DCE, last
            finish_test("PPT");
            model_enable = 1;
        end
    endtask

    task test_continuous;
        integer length, line;
        begin
            // Cover all three ending alignments, including single-word lines.
            for (length = 1; length <= 3 && length <= MAX_WORDS; length = length + 1) begin
                reset_dut;
                din_len = length;
                for (line = 0; line < 4; line = line + 1)
                    send_line(32'hfedcba98 ^ (line * 32'h13579bdf));
                drain;
            end
            // No idle between writes or lines; send waits only for din_hold.
            reset_dut;
            din_len = LONG_WORDS;
            for (line = 0; line < CONTINUOUS_LINES; line = line + 1)
                send_line(32'h89abcdef ^ (line * 32'h24681357));
            finish_test("CONTINUOUS");
        end
    endtask

    task test_random;
        integer frame, line, item;
        begin
            // Adjacent lines are equal length; reset starts a new length group.
            for (frame = 0; frame < 3; frame = frame + 1) begin
                reset_dut;
                read_mode = READ_RANDOM;
                din_len = (($random(write_seed) & 32'h7fffffff) % RANDOM_LIMIT) + 1;
                for (line = 0; line < RANDOM_LINES; line = line + 1)
                    for (item = 0; item < din_len; item = item + 1) begin
                        send($random(write_seed), item == din_len - 1);
                        idle($random(write_seed) & 3);
                    end
                drain;
            end
            finish_test("RANDOM");
        end
    endtask

    task test_srst;
        begin
            // Abort the first line. A reset must again require a full first line.
            reset_dut;
            din_len = 2;
            send(32'h89abcdef, 0);
            idle(2);
            pulse_srst;
            din_len = 1;
            send(32'h00002001, 1);
            send(32'h00004003, 1);
            send(32'h00006005, 1);
            drain;

            // Abort a paired word waiting behind output hold.
            read_mode = READ_HOLD;
            send(32'h00008007, 1);
            idle(4);
            pulse_srst;
            read_mode = READ_NORMAL;
            din_len = 2;
            send_line(32'h01234567);
            send_line(32'h89abcdef);
            send_line(32'hfedcba98);
            drain;

            // Abort an already valid last output while it is held.
            send_line(32'h76543210);
            idle(1);
            wait (dout_vd && dout_last);
            @(negedge clk);
            #1; read_mode = READ_HOLD; dout_hold = 1;
            repeat (3) @(negedge clk);
            if (!dout_vd || !dout_last) $fatal(1, "No held last before srst");
            pulse_srst;
            read_mode = READ_NORMAL;
            din_len = 1;
            send(32'hff123456, 1);
            send(32'hff789abc, 1);
            send(32'hff135246, 1);
            finish_test("SRST");
        end
    endtask

    // Main stimulus: enable each test independently with its parameter.
    initial begin
        if (DUMP_WAVE) begin
            $dumpfile("tb_repack_blk_V2.vcd");
            $dumpvars(0, tb_repack_blk_V2);
        end
        rstn = 0; srst = 0;
        din = 0; din_len = 2; din_last = 0; din_vd = 0; dout_hold = 0;
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
            $display("PASS RePack V2: %0d tests, depth=%0d seed=%0d", tests_passed, 1 << FIFO_ADDR_WIDTH, SEED);
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
            previous_valid = 0; sample_index = 0;
            write_index = 0; model_len = 0;
            expected_count = 0; received = 0;
            accepted = 0; last_count = 0;
            input_pending = 0; output_pending = 0;
            #1;
            if ({dout_vd, dout_last, dout} !== 26'd0)
                $fatal(1, "Output did not reset");
        end else begin
            if (input_pending && {din_len, din_vd, din_last, din} !== saved_input)
                $fatal(1, "Input changed before acceptance");
            if (output_pending && {dout_vd, dout_last, dout} !== saved_output)
                $fatal(1, "Output changed before hold release");
            input_pending = din_vd && din_hold;
            output_pending = dout_hold;
            saved_input = {din_len, din_vd, din_last, din};
            saved_output = {dout_vd, dout_last, dout};
            if (dout_vd !== 0 && dout_vd !== 1)
                $fatal(1, "Output valid is unknown");

            if (din_vd && !din_hold) begin
                accepted = accepted + 1;
                if (write_index == 0) begin
                    if (din_len == 0 || din_len > MAX_WORDS)
                        $fatal(1, "Invalid test line length");
                    if (previous_valid && din_len != model_len)
                        $fatal(1, "Adjacent test lines have different lengths");
                    model_len = din_len;
                end
                if (din_last !== (write_index == model_len - 1))
                    $fatal(1, "Input last disagrees with line length");
                write_index = din_last ? 0 : write_index + 1;
                if (model_enable) begin
                    for (bit_index = 0; bit_index < 32; bit_index = bit_index + 1) begin
                        model_word[model_bits] = din[bit_index];
                        model_bits = model_bits + 1;
                        if (model_bits == 12) begin
                            if (sample_index >= 10920) $fatal(1, "Reference line full");
                            if (previous_valid) begin
                                if (expected_count >= 65536) $fatal(1, "Expected queue full");
                                expected[expected_count] = {model_word, previous_line[sample_index]};
                                expected_last[expected_count] = 0;
                                expected_count = expected_count + 1;
                            end
                            previous_line[sample_index] = model_word;
                            sample_index = sample_index + 1;
                            model_word = 0; model_bits = 0;
                        end
                    end
                    if (din_last) begin
                        if (previous_valid) expected_last[expected_count-1] = 1;
                        previous_valid = 1; sample_index = 0;
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
        $fatal(1, "RePack V2 timeout");
    end
endmodule
