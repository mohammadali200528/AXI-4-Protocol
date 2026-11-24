// AXI4-Lite master (one-hot FSM), single outstanding transfer.

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