// Code your design here
// AXI4-Lite slave
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
    input  wire [3:0]  Wstr,      
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

    wire arr  = ~rv & ~wrco;           // ARready
    wire arf  = ARvalid & arr;         // ar_fire

    // Next-state inputs of the flip-flops
    wire d_awg = awf  | (aw & ~wrco);
    wire d_wg  = wf   | (w  & ~wrco);
    wire d_bv  = wrco | (bv & ~BReady);
    wire d_rv  = arf  | (rv & ~RReady);

    // select = wrco: 1 -> write address, 0 -> read address
    wire [5:0] A = wrco ? addr_reg[7:2] : ARAddr[7:2];

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
    assign BResp   = 2'b00;            

    assign ARready = arr;
    assign Rvalid  = rv;
    assign RData   = rdata_reg;
    assign RResp   = 2'b00;           

endmodule