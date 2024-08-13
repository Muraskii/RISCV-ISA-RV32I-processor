module WB_Mux (
    input [2:0] wb_mux_sel_reg_in, // Input selector
    input alu_src_reg_in, // ALU source selector

    input [31:0] alu_result_in,
    input [31:0] lu_output_in,
    input [31:0] imm_reg_in,
    input [31:0] csr_data_in,
    input [31:0] pc_plus_4_reg_in,
    input [31:0] iadder_out_reg_in,
    input [31:0] rs2_reg_in,

    output reg [31:0] wb_mux_out,
    output reg [31:0] alu_2nd_src_mux_out
);

// Parameter definitions for wb_mux_sel_reg_in
localparam WB_ALU = 3'b000;
localparam WB_LU = 3'b001;
localparam WB_IMM = 3'b010;
localparam WB_CSR = 3'b011;
localparam WB_PC_PLUS = 3'b100;
localparam WB_IADDER_OUT = 3'b101;

always @(*) begin
    // wb_mux_out logic
    case (wb_mux_sel_reg_in)
        WB_ALU: wb_mux_out = alu_result_in;
        WB_LU: wb_mux_out = lu_output_in;
        WB_IMM: wb_mux_out = imm_reg_in;
        WB_CSR: wb_mux_out = csr_data_in;
        WB_PC_PLUS: wb_mux_out = pc_plus_4_reg_in;
        WB_IADDER_OUT: wb_mux_out = iadder_out_reg_in;
        default: wb_mux_out = alu_result_in; // Default case
    endcase

    // alu_2nd_src_mux_out logic
    if (alu_src_reg_in)
        alu_2nd_src_mux_out = rs2_reg_in;
    else
        alu_2nd_src_mux_out = imm_reg_in;
end

endmodule