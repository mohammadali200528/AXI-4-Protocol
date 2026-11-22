`timescale 1ns/1ps

module tb_write4_slave1;

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

    task write(input [7:0] a, input [31:0] d);
        begin
            $display("WRITE addr=%h data=%h", a, d);
            @(negedge Clk);
            req = 1; we = 1; addr = a; wdata = d; wstrb = 4'hF;
            @(negedge Clk);
            req = 0;
            wait_done;
            check({30'd0, resp}, 32'd0, "write response (00 = OKAY)");
            @(negedge Clk);
        end
    endtask

    // ---------------- 5. the test ----------------
    initial begin
        $dumpfile("dump_write4.vcd");
        $dumpvars(0, tb_write4_slave1);

        for (i = 0; i < 64; i = i + 1) begin
            dut.u_slave0.mem[i] = 32'd0;
            dut.u_slave1.mem[i] = 32'd0;
        end

        $display("\n=== reset ===");
        do_reset;

        $display("\n=== WRITE TEST 4: 9ABCDEF0 to address 90 (slave 1) ===");
        write(8'h90, 32'h9ABCDEF0);

        // slave 1 must hold the data
        check(dut.u_slave1.mem[36], 32'h9ABCDEF0, "slave 1 word 36");

        // slave 0 must be untouched (every word still 0)
        for (i = 0; i < 64; i = i + 1)
            check(dut.u_slave0.mem[i], 32'h00000000, "slave 0 word (untouched)");

        $display("\n----------------------------------------");
        if (errors == 0) $display("WRITE TEST 4: PASSED");
        else             $display("WRITE TEST 4: %0d CHECK(S) FAILED", errors);
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