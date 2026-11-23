// ============================================================================
// axi_lite_design.sv  -  COMBINED DESIGN FILE (all three design modules)
//
//   1. axi_lite_slave   RAM slave (64 x 32-bit words)
//   2. axi_lite_master  one-hot FSM master (user side: req/we/addr/wdata/wstrb)
//   3. axi_lite_top     1 master + 2 slaves, address decode on addr[7]
//                         slave 0 = 0x00-0x7F, slave 1 = 0x80-0xFF
//
// ============================================================================

// ----------------------------------------------------------------------------
// 1. SLAVE
// ----------------------------------------------------------------------------
// AXI4-Lite slave, RAM based (64 x 32-bit words)
// Signal and internal names match the Logisim "slave" circuit:
//   aw, w, bv, rv        = aw_got, w_got, bvalid, rvalid flip-flops
//   awr, wrd, wrco       = AWready, Wready, wr_commit
//   awf, wf, arr, arf    = aw_fire, w_fire, ARready, ar_fire
//   d_awg, d_wg, d_bv, d_rv = D inputs of the four flip-flops

module axi_lite_slave (
    input  wire        Clk,
    input  wire        Rst_n,

    // Write address channel
    input  wire [7:0]  AWAddr,
    input  wire        AWvalid,
    output wire        AWready,

    // Write data channel
    input  wire [31:0] WData,
    input  wire [3:0]  Wstr,      // not used yet (full-word writes only)
    input  wire        WValid,
    output wire        Wready,

    // Write response channel
    output wire [1:0]  BResp,
    output wire        Bvalid,
    input  wire        BReady,

    // Read address channel
    input  wire [7:0]  ARAddr,
    input  wire        ARvalid,
    output wire        ARready,

    // Read data channel
    output wire [31:0] RData,
    output wire [1:0]  RResp,
    output wire        Rvalid,
    input  wire        RReady
);

    // ---------------- State ----------------
    reg        aw, w, bv, rv;          // aw_got, w_got, bvalid, rvalid
    reg [7:0]  addr_reg;
    reg [31:0] data_reg;
    reg [31:0] rdata_reg;

    // ---------------- RAM (64 words) ----------------
    reg [31:0] mem [0:63];

    // ---------------- Handshake logic ----------------
    wire awr  = ~aw & ~bv;             // AWready
    wire wrd  = ~w  & ~bv;             // Wready
    wire awf  = AWvalid & awr;         // aw_fire
    wire wf   = WValid  & wrd;         // w_fire
    wire wrco = aw & w & ~bv;          // wr_commit

    wire arr  = ~rv & ~wrco;           // ARready (RAM has one address port)
    wire arf  = ARvalid & arr;         // ar_fire

    // Next-state inputs of the flip-flops
    wire d_awg = awf  | (aw & ~wrco);
    wire d_wg  = wf   | (w  & ~wrco);
    wire d_bv  = wrco | (bv & ~BReady);
    wire d_rv  = arf  | (rv & ~RReady);

    // ---------------- RAM address mux ----------------
    // select = wrco: 1 -> write address, 0 -> read address
    wire [5:0] A = wrco ? addr_reg[7:2] : ARAddr[7:2];

    // ---------------- Flip-flops and registers ----------------
    // Logisim 'clr' is asynchronous, so reset is asynchronous here too.
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            aw        <= 1'b0;
            w         <= 1'b0;
            bv        <= 1'b0;
            rv        <= 1'b0;
            addr_reg  <= 8'd0;
            data_reg  <= 32'd0;
            rdata_reg <= 32'd0;
        end else begin
            aw <= d_awg;
            w  <= d_wg;
            bv <= d_bv;
            rv <= d_rv;

            if (awf) addr_reg  <= AWAddr;     // enable = aw_fire
            if (wf)  data_reg  <= WData;      // enable = w_fire
            if (arf) rdata_reg <= mem[A];     // enable = ar_fire
        end
    end

    // RAM write on wr_commit (store port)
    always @(posedge Clk) begin
        if (wrco)
            mem[A] <= data_reg;
    end

    // ---------------- Outputs ----------------
    assign AWready = awr;
    assign Wready  = wrd;
    assign Bvalid  = bv;
    assign BResp   = 2'b00;            // OKAY

    assign ARready = arr;
    assign Rvalid  = rv;
    assign RData   = rdata_reg;
    assign RResp   = 2'b00;            // OKAY

endmodule


// ----------------------------------------------------------------------------
// 2. MASTER
// ----------------------------------------------------------------------------
// AXI4-Lite master (one-hot FSM), single outstanding transfer.
// Pin and internal names match the Logisim "master" circuit:
//   wr, wrr, rda, rdd, dn = state flip-flops (WR, WR_RESP, RD_ADDR, RD_DATA, DONE)
//   awd, wd               = AW accepted / W accepted flags
//   idle                  = NOR of the five state flops (no IDLE flop)
//
// User side (driven by hand now, by the core later):
//   req   : pulse high for ONE clock to start a transfer
//   we    : 1 = write, 0 = read
//   addr, wdata, wstrb : latched when the transfer starts
//   done  : high for one clock when the transfer has finished
//   rdata : read data (valid while done is high, and held afterwards)
//   resp  : AXI response of the last transfer (00 = OKAY)

module axi_lite_master (
    input  wire        Clk,
    input  wire        Rst_n,

    // ---- user side ----
    input  wire        req,
    input  wire        we,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    output wire        done,
    output wire [31:0] rdata,
    output wire [1:0]  resp,

    // ---- AXI write address channel ----
    output wire [7:0]  AWAddr,
    output wire        AWvalid,
    input  wire        AWready,

    // ---- AXI write data channel ----
    output wire [31:0] WData,
    output wire [3:0]  Wstr,
    output wire        WValid,
    input  wire        Wready,

    // ---- AXI write response channel ----
    input  wire [1:0]  BResp,
    input  wire        Bvalid,
    output wire        BReady,

    // ---- AXI read address channel ----
    output wire [7:0]  ARAddr,
    output wire        ARvalid,
    input  wire        ARready,

    // ---- AXI read data channel ----
    input  wire [31:0] RData,
    input  wire [1:0]  RResp,
    input  wire        Rvalid,
    output wire        RReady
);

    // ---------------- State flip-flops (one-hot) ----------------
    reg wr, wrr, rda, rdd, dn;     // WR, WR_RESP, RD_ADDR, RD_DATA, DONE
    reg awd, wd;                   // AW accepted, W accepted

    // ---------------- IDLE and start ----------------
    wire idle    = ~(wr | wrr | rda | rdd | dn);
    wire start   = idle & req;
    wire start_w = start & we;
    wire start_r = start & ~we;

    // ---------------- AW / W accepted flags ----------------
    wire aw_ok   = awd | AWready;
    wire w_ok    = wd  | Wready;
    wire both_ok = aw_ok & w_ok;

    wire d_awd = wr & aw_ok & ~w_ok;
    wire d_wd  = wr & w_ok  & ~aw_ok;

    // ---------------- Next-state logic ----------------
    wire wr_hold  = wr  & ~both_ok;
    wire wr_go    = wr  &  both_ok;
    wire wrr_hold = wrr & ~Bvalid;
    wire b_got    = wrr &  Bvalid;
    wire rda_hold = rda & ~ARready;
    wire rda_go   = rda &  ARready;
    wire rdd_hold = rdd & ~Rvalid;
    wire r_got    = rdd &  Rvalid;

    wire d_wr  = start_w | wr_hold;
    wire d_wrr = wr_go   | wrr_hold;
    wire d_rda = start_r | rda_hold;
    wire d_rdd = rda_go  | rdd_hold;
    wire d_dn  = b_got   | r_got;

    // ---------------- Registers ----------------
    reg [7:0]  addr_l;
    reg [31:0] wdata_l;
    reg [3:0]  wstrb_l;
    reg [31:0] rdata_reg;
    reg [1:0]  resp_reg;

    // Logisim 'clr' is asynchronous, so reset is asynchronous here too.
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            wr  <= 1'b0;  wrr <= 1'b0;  rda <= 1'b0;
            rdd <= 1'b0;  dn  <= 1'b0;
            awd <= 1'b0;  wd  <= 1'b0;
            addr_l    <= 8'd0;
            wdata_l   <= 32'd0;
            wstrb_l   <= 4'd0;
            rdata_reg <= 32'd0;
            resp_reg  <= 2'd0;
        end else begin
            wr  <= d_wr;   wrr <= d_wrr;  rda <= d_rda;
            rdd <= d_rdd;  dn  <= d_dn;
            awd <= d_awd;  wd  <= d_wd;

            if (start) begin                    // latch request (enable = start)
                addr_l  <= addr;
                wdata_l <= wdata;
                wstrb_l <= wstrb;
            end
            if (r_got) rdata_reg <= RData;      // enable = r_got
            if (d_dn)  resp_reg  <= wrr ? BResp : RResp;   // enable = d_dn
        end
    end

    // ---------------- Outputs ----------------
    assign AWvalid = wr & ~awd;
    assign WValid  = wr & ~wd;
    assign BReady  = wrr;
    assign ARvalid = rda;
    assign RReady  = rdd;
    assign done    = dn;

    assign AWAddr  = addr_l;
    assign ARAddr  = addr_l;
    assign WData   = wdata_l;
    assign Wstr    = wstrb_l;

    assign rdata   = rdata_reg;
    assign resp    = resp_reg;

endmodule


// ----------------------------------------------------------------------------
// 3. TOP SHEET  (1 master + 2 slaves)
// ----------------------------------------------------------------------------
// Top sheet: 1 AXI4-Lite master + 2 RAM slaves with address decode.
// Names match the Logisim top sheet (M_*, s0_*, s1_*, selw, selr).
//
// Address map (8-bit byte address):
//   slave 0 : 0x00 - 0x7F   (addr[7] = 0)
//   slave 1 : 0x80 - 0xFF   (addr[7] = 1)
//
// Each slave is a 64-word RAM indexed by addr[7:2], so slave 0 only ever
// uses words 0-31 and slave 1 only words 32-63 (half of each RAM is unused).
//
// How the interconnect works:
//   * address, write data and strobes go to BOTH slaves (shared buses);
//   * the master's VALIDs and its BReady/RReady are ANDed with the decode,
//     so only the addressed slave sees a request;
//   * the slaves' READYs, responses and read data come back through muxes
//     selected by the same decode.
//   The select is stable for a whole transfer because AWAddr/ARAddr come
//   from the master's latched address register.

module axi_lite_top (
    input  wire        Clk,
    input  wire        Rst_n,

    // user side of the master
    input  wire        req,       // pulse high for ONE clock to start
    input  wire        we,        // 1 = write, 0 = read
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    output wire        done,      // one-clock pulse when the transfer finishes
    output wire [31:0] rdata,
    output wire [1:0]  resp
);

    // ---------------- master-side AXI wires (M_*) ----------------
    wire [7:0]  M_AWAddr;
    wire        M_AWvalid, M_AWready;
    wire [31:0] M_WData;
    wire [3:0]  M_Wstr;
    wire        M_WValid,  M_Wready;
    wire [1:0]  M_BResp;
    wire        M_Bvalid,  M_BReady;
    wire [7:0]  M_ARAddr;
    wire        M_ARvalid, M_ARready;
    wire [31:0] M_RData;
    wire [1:0]  M_RResp;
    wire        M_Rvalid,  M_RReady;

    // ---------------- slave 0 wires (s0_*) ----------------
    wire        s0_AWvalid, s0_AWready;
    wire        s0_WValid,  s0_Wready;
    wire [1:0]  s0_BResp;
    wire        s0_Bvalid,  s0_BReady;
    wire        s0_ARvalid, s0_ARready;
    wire [31:0] s0_RData;
    wire [1:0]  s0_RResp;
    wire        s0_Rvalid,  s0_RReady;

    // ---------------- slave 1 wires (s1_*) ----------------
    wire        s1_AWvalid, s1_AWready;
    wire        s1_WValid,  s1_Wready;
    wire [1:0]  s1_BResp;
    wire        s1_Bvalid,  s1_BReady;
    wire        s1_ARvalid, s1_ARready;
    wire [31:0] s1_RData;
    wire [1:0]  s1_RResp;
    wire        s1_Rvalid,  s1_RReady;

    // ---------------- master ----------------
    axi_lite_master u_master (
        .Clk(Clk),            .Rst_n(Rst_n),
        .req(req),            .we(we),
        .addr(addr),          .wdata(wdata),         .wstrb(wstrb),
        .done(done),          .rdata(rdata),         .resp(resp),

        .AWAddr(M_AWAddr),    .AWvalid(M_AWvalid),   .AWready(M_AWready),
        .WData(M_WData),      .Wstr(M_Wstr),         .WValid(M_WValid),   .Wready(M_Wready),
        .BResp(M_BResp),      .Bvalid(M_Bvalid),     .BReady(M_BReady),
        .ARAddr(M_ARAddr),    .ARvalid(M_ARvalid),   .ARready(M_ARready),
        .RData(M_RData),      .RResp(M_RResp),       .Rvalid(M_Rvalid),   .RReady(M_RReady)
    );

    // ---------------- address decode ----------------
    wire selw = M_AWAddr[7];     // 0 -> slave 0, 1 -> slave 1 (write side)
    wire selr = M_ARAddr[7];     // 0 -> slave 0, 1 -> slave 1 (read side)

    // ---------------- valid / ready gating (master -> slaves) ----------------
    assign s0_AWvalid = M_AWvalid & ~selw;
    assign s1_AWvalid = M_AWvalid &  selw;
    assign s0_WValid  = M_WValid  & ~selw;
    assign s1_WValid  = M_WValid  &  selw;
    assign s0_BReady  = M_BReady  & ~selw;
    assign s1_BReady  = M_BReady  &  selw;
    assign s0_ARvalid = M_ARvalid & ~selr;
    assign s1_ARvalid = M_ARvalid &  selr;
    assign s0_RReady  = M_RReady  & ~selr;
    assign s1_RReady  = M_RReady  &  selr;

    // ---------------- response muxes (slaves -> master) ----------------
    assign M_AWready = selw ? s1_AWready : s0_AWready;
    assign M_Wready  = selw ? s1_Wready  : s0_Wready;
    assign M_Bvalid  = selw ? s1_Bvalid  : s0_Bvalid;
    assign M_BResp   = selw ? s1_BResp   : s0_BResp;

    assign M_ARready = selr ? s1_ARready : s0_ARready;
    assign M_Rvalid  = selr ? s1_Rvalid  : s0_Rvalid;
    assign M_RData   = selr ? s1_RData   : s0_RData;
    assign M_RResp   = selr ? s1_RResp   : s0_RResp;

    // ---------------- slave 0 : 0x00 - 0x7F ----------------
    axi_lite_slave u_slave0 (
        .Clk(Clk),            .Rst_n(Rst_n),
        .AWAddr(M_AWAddr),    .AWvalid(s0_AWvalid),  .AWready(s0_AWready),
        .WData(M_WData),      .Wstr(M_Wstr),         .WValid(s0_WValid),  .Wready(s0_Wready),
        .BResp(s0_BResp),     .Bvalid(s0_Bvalid),    .BReady(s0_BReady),
        .ARAddr(M_ARAddr),    .ARvalid(s0_ARvalid),  .ARready(s0_ARready),
        .RData(s0_RData),     .RResp(s0_RResp),      .Rvalid(s0_Rvalid),  .RReady(s0_RReady)
    );

    // ---------------- slave 1 : 0x80 - 0xFF ----------------
    axi_lite_slave u_slave1 (
        .Clk(Clk),            .Rst_n(Rst_n),
        .AWAddr(M_AWAddr),    .AWvalid(s1_AWvalid),  .AWready(s1_AWready),
        .WData(M_WData),      .Wstr(M_Wstr),         .WValid(s1_WValid),  .Wready(s1_Wready),
        .BResp(s1_BResp),     .Bvalid(s1_Bvalid),    .BReady(s1_BReady),
        .ARAddr(M_ARAddr),    .ARvalid(s1_ARvalid),  .ARready(s1_ARready),
        .RData(s1_RData),     .RResp(s1_RResp),      .Rvalid(s1_Rvalid),  .RReady(s1_RReady)
    );

endmodule