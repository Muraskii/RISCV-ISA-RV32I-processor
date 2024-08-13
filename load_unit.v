module load_unit (
    input ahb_resp_in,
    input [31:0] ms_riscv32_mp_dmdata_in,
    input [1:0] iadder_out_1_to_0_in,
    input load_unsigned_in,
    input [1:0] load_size_in,
    output reg [31:0] lu_output_out
);

// Initialize output register
initial begin
    lu_output_out = 32'b0;
end

always @(*) begin
    if (!ahb_resp_in) begin
        case (load_size_in)
            2'b00: begin // LB or LBU - Load Byte
                case (iadder_out_1_to_0_in)
                    2'b00: lu_output_out = load_unsigned_in ? {24'b0, ms_riscv32_mp_dmdata_in[7:0]} : {{24{ms_riscv32_mp_dmdata_in[7]}}, ms_riscv32_mp_dmdata_in[7:0]};
                    2'b01: lu_output_out = load_unsigned_in ? {24'b0, ms_riscv32_mp_dmdata_in[15:8]} : {{24{ms_riscv32_mp_dmdata_in[15]}}, ms_riscv32_mp_dmdata_in[15:8]};
                    2'b10: lu_output_out = load_unsigned_in ? {24'b0, ms_riscv32_mp_dmdata_in[23:16]} : {{24{ms_riscv32_mp_dmdata_in[23]}}, ms_riscv32_mp_dmdata_in[23:16]};
                    2'b11: lu_output_out = load_unsigned_in ? {24'b0, ms_riscv32_mp_dmdata_in[31:24]} : {{24{ms_riscv32_mp_dmdata_in[31]}}, ms_riscv32_mp_dmdata_in[31:24]};
                    default: lu_output_out = 32'b0; // Default case for unexpected values
                endcase
            end
            2'b01: begin // LH or LHU - Load Halfword
                case (iadder_out_1_to_0_in[1])
                    1'b0: lu_output_out = load_unsigned_in ? {16'b0, ms_riscv32_mp_dmdata_in[15:0]} : {{16{ms_riscv32_mp_dmdata_in[15]}}, ms_riscv32_mp_dmdata_in[15:0]};
                    1'b1: lu_output_out = load_unsigned_in ? {16'b0, ms_riscv32_mp_dmdata_in[31:16]} : {{16{ms_riscv32_mp_dmdata_in[31]}}, ms_riscv32_mp_dmdata_in[31:16]};
                    default: lu_output_out = 32'b0; // Default case for unexpected values
                endcase
            end
            2'b10, 2'b11: begin // LW - Load Word
                lu_output_out = ms_riscv32_mp_dmdata_in;
            end
            default: lu_output_out = 32'b0; // Default case for unexpected values
        endcase
    end else begin
        lu_output_out = 32'b0; // Default when ahb_resp_in is active
    end
end

endmodule
