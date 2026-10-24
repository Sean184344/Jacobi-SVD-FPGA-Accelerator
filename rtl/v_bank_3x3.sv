// v_bank_3x3.sv
// Tracks the right singular vector matrix V.
// Initialized to identity (I), then each Jacobi rotation J is accumulated:
//   V <- V * J
// achieved by applying the same column rotation (same c, s, same column
// indices) to V's own columns -- NOT to A's rotated columns.
// At the end, V contains the right singular vectors of A.

module v_bank_3x3 (
    input  logic clk,
    input  logic rst,
    input  logic write_enable,
    input  logic [1:0] col_i_idx,
    input  logic [1:0] col_j_idx,
    input  logic signed [31:0] col_i_new [0:2],  // connect to V's column rotator output
    input  logic signed [31:0] col_j_new [0:2],  // connect to V's column rotator output
    output logic signed [31:0] col_i     [0:2],  // V's current col_i for the rotator
    output logic signed [31:0] col_j     [0:2],  // V's current col_j for the rotator
    output logic signed [31:0] V_out     [0:2][0:2]
);

logic signed [31:0] V [0:2][0:2];

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        // Start from identity (Q16.16: 1.0 = 65536)
        for (int r = 0; r < 3; r++)
            for (int cc = 0; cc < 3; cc++)
                V[r][cc] <= (r == cc) ? 32'sd65536 : 32'sd0;
    end else if (write_enable) begin
        for (int r = 0; r < 3; r++) begin
            V[r][col_i_idx] <= col_i_new[r];
            V[r][col_j_idx] <= col_j_new[r];
        end
    end
end

always_comb begin
    for (int r = 0; r < 3; r++) begin
        col_i[r] = V[r][col_i_idx];
        col_j[r] = V[r][col_j_idx];
    end
    for (int r = 0; r < 3; r++)
        for (int cc = 0; cc < 3; cc++)
            V_out[r][cc] = V[r][cc];
end

endmodule