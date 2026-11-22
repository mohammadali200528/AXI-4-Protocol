`timescale 1ns/1ps

module tb_read4_slave1;

    // ---------------- 1. signals + DUT ----------------
    reg         Clk = 0;
    reg         Rst_n = 0;
    reg         req = 0, we = 0;
    reg  [7:0]  addr = 0;
    reg  [31:0] wdata = 0;
    reg  [3:0]  wstrb = 4'hF;
    wire        done;
    wire [31:0] rdata;
    wire [1:0]  resp;

    axi_lite_top dut (
        .Clk(Clk), .Rst_n(Rst_n),
        .req(req), .we(we), .addr(addr), .wdata(wdata), .wstrb(wstrb),
        .done(done), .rdata(rdata), .resp(resp)
    );

    // ---------------- 2. clock ----------------
    always #5 Clk = ~Clk;

    // ---------------- 3. bookkeeping ----------------
    integer errors = 0;
    integer i;

    // ---------------- 4. helper tasks ----------------
    task check(input [31:0] got, input [31:0] exp, input [8*40-1:0] name);
        begin
            if (got === exp)
                $display("   [PASS] %0s = %h", name, got);
            else begin
                errors = errors + 1;
                $display("   [FAIL] %0s: got %h, expected %h", name, got, exp);
            end
        end
    endtask

    task do_reset;
        begin
            @(negedge Clk); Rst_n = 0;
            repeat (2) @(negedge Clk);
            Rst_n = 1;
            @(negedge Clk);
        end
    endtask

    task wait_done;
        integer t;
        begin
            t = 0;
            while (!done && t < 50) begin
                @(negedge Clk);
                t = t + 1;
            end
            if (!done) begin
                errors = errors + 1;
                $display("   [FAIL] done never rose (timeout)");
            end
        end
    endtask

    // One read: req is high for exactly ONE clock, then compare rdata
    task read(input [7:0] a, input [31:0] exp);
        begin
            $display("READ  addr=%h", a);
            @(negedge Clk);
            req = 1; we = 0; addr = a;
            @(negedge Clk);
            req = 0;
            wait_done;
            check(rdata, exp, "rdata");
            check({30'd0, resp}, 32'd0, "read response (00 = OKAY)");
            @(negedge Clk);
        end
    endtask

    // All 64 words of a slave must be zero (one summary line)
    task check_all_zero(input integer slave, input [8*40-1:0] name);
        integer nonzero;
        reg [31:0] got;
        begin
            nonzero = 0;
            for (i = 0; i < 64; i = i + 1) begin
                if (slave == 0) got = dut.u_slave0.mem[i];
                else            got = dut.u_slave1.mem[i];
                if (got !== 32'd0) nonzero = nonzero + 1;
            end
            if (nonzero == 0)
                $display("   [PASS] %0s (all 64 words are 0)", name);
            else begin
                errors = errors + 1;
                $display("   [FAIL] %0s: %0d words are not 0", name, nonzero);
            end
        end
    endtask

    // ---------------- 5. the test ----------------
    initial begin
        $dumpfile("dump_read4.vcd");
        $dumpvars(0, tb_read4_slave1);

        // clear both RAMs
        for (i = 0; i < 64; i = i + 1) begin
            dut.u_slave0.mem[i] = 32'd0;
            dut.u_slave1.mem[i] = 32'd0;
        end

        $display("\n=== reset ===");
        do_reset;

        // preload the data we want to read (address C0 -> slave 1, word 48)
        dut.u_slave1.mem[48] = 32'hFEEDFACE;

        $display("\n=== READ TEST 4: read address C0 (slave 1) ===");
        read(8'hC0, 32'hFEEDFACE);

        // a read must not change any memory
        check(dut.u_slave1.mem[48], 32'hFEEDFACE, "slave 1 word 48 unchanged");
        check_all_zero(0, "slave 0 RAM untouched");

        $display("\n----------------------------------------");
        if (errors == 0) $display("READ TEST 4: PASSED");
        else             $display("READ TEST 4: %0d CHECK(S) FAILED", errors);
        $display("----------------------------------------");
        $finish;
    end

    // ---------------- 6. watchdog ----------------
    initial begin
        #100000;
        $display("TIMEOUT: testbench hung");
        $finish;
    end

endmodule