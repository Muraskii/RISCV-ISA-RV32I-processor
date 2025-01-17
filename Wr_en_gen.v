module Wr_en_gen (
    input wire csr_wr_en_reg_in,
    input wire rf_wr_en_reg_in,
    input wire flush_in,
    output wire wr_en_csr_file_out,
    output wire wr_en_integer_file_out
);

    // Mux to select between csr_wr_en_reg_in and 0 based on flush_in
    assign wr_en_csr_file_out = (flush_in) ? 1'b0 : csr_wr_en_reg_in;

    // Mux to select between rf_wr_en_reg_in and 0 based on flush_in
    assign wr_en_integer_file_out = (flush_in) ? 1'b0 : rf_wr_en_reg_in;

endmodule

