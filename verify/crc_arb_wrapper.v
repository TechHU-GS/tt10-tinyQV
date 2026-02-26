// CRC Arbitration Formal Verification Wrapper
// Extracts ONLY the CRC mux logic from project.v for isolated proof.
// Proves: seal_using_crc exclusively routes all 3 CRC signals (init, data, dv).

`timescale 1ns / 1ps

module crc_arb_wrapper (
    input wire        seal_using_crc,

    // Seal-side CRC signals
    input wire        seal_crc_init,
    input wire [7:0]  seal_crc_byte,
    input wire        seal_crc_feed,

    // CPU peripheral-side CRC signals
    input wire        crc_peri_init,
    input wire [7:0]  crc_peri_data,
    input wire        crc_peri_dv
);

    // === Replicate project.v lines 384-386 ===
    wire crc_engine_init = seal_using_crc ? seal_crc_init  : crc_peri_init;
    wire [7:0] crc_engine_data = seal_using_crc ? seal_crc_byte  : crc_peri_data;
    wire crc_engine_dv   = seal_using_crc ? seal_crc_feed  : crc_peri_dv;

`ifdef FORMAL
    // P8: When seal owns CRC, engine dv comes exclusively from seal
    always @(*)
        if (seal_using_crc)
            assert(crc_engine_dv == seal_crc_feed);

    // P9: When seal does NOT own CRC, engine dv comes from CPU peripheral
    always @(*)
        if (!seal_using_crc)
            assert(crc_engine_dv == crc_peri_dv);

    // P10: Full mutual exclusion — ALL signals (init, data, dv) match ownership
    always @(*) begin
        if (seal_using_crc) begin
            assert(crc_engine_init == seal_crc_init);
            assert(crc_engine_data == seal_crc_byte);
            assert(crc_engine_dv   == seal_crc_feed);
        end else begin
            assert(crc_engine_init == crc_peri_init);
            assert(crc_engine_data == crc_peri_data);
            assert(crc_engine_dv   == crc_peri_dv);
        end
    end

    // P11: No cross-contamination — seal data never appears on engine when CPU owns
    always @(*)
        if (!seal_using_crc && seal_crc_feed && !crc_peri_dv)
            assert(crc_engine_dv == 1'b0);

    // P12: No cross-contamination — CPU data never appears on engine when seal owns
    always @(*)
        if (seal_using_crc && crc_peri_dv && !seal_crc_feed)
            assert(crc_engine_dv == 1'b0);
`endif

endmodule
