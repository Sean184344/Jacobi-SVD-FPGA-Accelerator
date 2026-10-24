module jacobi_svd_3x3 #(
    parameter MAX_ROTATIONS = 108,  // ~12 sweeps; increased for ill-conditioned matrices
    parameter CONV_SHIFT    = 18    // relative convergence: |gamma| < max_norm >> CONV_SHIFT
)(
    input  logic clk,
    input  logic rst,
    input  logic signed [31:0] matrix_in  [0:2][0:2],
    output logic signed [31:0] matrix_out [0:2][0:2],  // final A*V = U*Sigma
    output logic signed [31:0] V_out      [0:2][0:2],  // right singular vectors
    output logic svd_done
);

localparam int ACC_W = 2*32 + $clog2(3); // = 66, matches jacobi_params

//------------------------------------
// Internal signals
//------------------------------------
logic [1:0] col_i_idx, col_j_idx;
logic       start, rotation_done, write_enable;
logic [5:0] rotation_count;
logic signed [31:0] c_q16_rot, s_q16_rot; // cos/sin from the rotation engine

assign write_enable = rotation_done;

// A matrix columns selected for this rotation
logic signed [31:0] col_i_sel [0:2];
logic signed [31:0] col_j_sel [0:2];
logic signed [31:0] col_i_rot [0:2];
logic signed [31:0] col_j_rot [0:2];

// V matrix columns for this rotation
logic signed [31:0] v_col_i_cur [0:2];
logic signed [31:0] v_col_j_cur [0:2];
logic signed [31:0] v_col_i_rot [0:2];
logic signed [31:0] v_col_j_rot [0:2];

logic signed [31:0] mat_int [0:2][0:2];
assign matrix_out = mat_int;

//------------------------------------
// A Matrix Bank
//------------------------------------
matrix_bank_3x3 MEM (
    .clk(clk), .rst(rst),
    .write_enable(write_enable),
    .col_i_idx(col_i_idx), .col_j_idx(col_j_idx),
    .col_i_new(col_i_rot), .col_j_new(col_j_rot),
    .matrix_init(matrix_in),
    .col_i(col_i_sel),     .col_j(col_j_sel),
    .matrix_out(mat_int)
);

//------------------------------------
// V Matrix Bank (right singular vectors, starts as identity)
//------------------------------------
v_bank_3x3 VBANK (
    .clk(clk), .rst(rst),
    .write_enable(write_enable),
    .col_i_idx(col_i_idx), .col_j_idx(col_j_idx),
    .col_i_new(v_col_i_rot), .col_j_new(v_col_j_rot),
    .col_i(v_col_i_cur),     .col_j(v_col_j_cur),
    .V_out(V_out)
);

//------------------------------------
// V Column Rotator
// Applies the SAME c_q16/s_q16 to V's columns so V accumulates V <- V*J.
// Note: uses c_q16_rot/s_q16_rot from jacobi_rotation (stable after
// sincos_valid fires, remains stable through the DONE cycle when
// write_enable fires).
//------------------------------------
column_rotator #(.N(3), .WIDTH(32), .SCALE(65536)) V_ROT (
    .c(c_q16_rot), .s(s_q16_rot),
    .col_i(v_col_i_cur), .col_j(v_col_j_cur),
    .col_i_rot(v_col_i_rot), .col_j_rot(v_col_j_rot)
);

//------------------------------------
// Extract all three columns from mat_int for greedy pair selection.
// These update combinationally every cycle -- always reflects the live matrix.
//------------------------------------
logic signed [31:0] col0 [0:2], col1 [0:2], col2 [0:2];

genvar gr;
generate
    for (gr = 0; gr < 3; gr++) begin : col_ext
        assign col0[gr] = mat_int[gr][0];
        assign col1[gr] = mat_int[gr][1];
        assign col2[gr] = mat_int[gr][2];
    end
endgenerate

//------------------------------------
// Three simultaneous jacobi_params: one per pair.
//------------------------------------
logic signed [ACC_W-1:0] alpha_01, beta_01, gamma_01;
logic signed [ACC_W-1:0] alpha_02, beta_02, gamma_02;
logic signed [ACC_W-1:0] alpha_12, beta_12, gamma_12;

jacobi_params #(.N(3), .WIDTH(32)) P01 (
    .col_i(col0), .col_j(col1),
    .alpha(alpha_01), .beta(beta_01), .gamma(gamma_01)
);
jacobi_params #(.N(3), .WIDTH(32)) P02 (
    .col_i(col0), .col_j(col2),
    .alpha(alpha_02), .beta(beta_02), .gamma(gamma_02)
);
jacobi_params #(.N(3), .WIDTH(32)) P12 (
    .col_i(col1), .col_j(col2),
    .alpha(alpha_12), .beta(beta_12), .gamma(gamma_12)
);

//------------------------------------
// Greedy pair selector
//------------------------------------
logic signed [ACC_W-1:0] abs_g01, abs_g02, abs_g12;
assign abs_g01 = (gamma_01 < 0) ? -gamma_01 : gamma_01;
assign abs_g02 = (gamma_02 < 0) ? -gamma_02 : gamma_02;
assign abs_g12 = (gamma_12 < 0) ? -gamma_12 : gamma_12;

logic [1:0] best_i, best_j;
always_comb begin
    if (abs_g01 >= abs_g02 && abs_g01 >= abs_g12) begin
        best_i = 2'd0; best_j = 2'd1;
    end else if (abs_g02 >= abs_g12) begin
        best_i = 2'd0; best_j = 2'd2;
    end else begin
        best_i = 2'd1; best_j = 2'd2;
    end
end

//------------------------------------
// Scale-invariant convergence check
//------------------------------------
logic signed [ACC_W-1:0] max_norm;
always_comb begin
    max_norm = alpha_01;
    if (beta_01  > max_norm) max_norm = beta_01;
    if (beta_12  > max_norm) max_norm = beta_12;
    if (alpha_12 > max_norm) max_norm = alpha_12;
end

logic signed [ACC_W-1:0] conv_threshold;
assign conv_threshold = max_norm >>> CONV_SHIFT;

logic all_converged;
assign all_converged = (abs_g01 <= conv_threshold) &&
                       (abs_g02 <= conv_threshold) &&
                       (abs_g12 <= conv_threshold);

//------------------------------------
// Sweep Controller (rotation sequencer)
//------------------------------------
jacobi_sweep_controller #(.MAX_ROTATIONS(MAX_ROTATIONS)) CTRL (
    .clk(clk), .rst(rst),
    .rotation_done(rotation_done),
    .all_converged(all_converged),
    .best_i(best_i), .best_j(best_j),
    .col_i(col_i_idx), .col_j(col_j_idx),
    .rotation_count(rotation_count),
    .start(start), .done(svd_done)
);

//------------------------------------
// Jacobi Rotation Engine (processes A columns)
//------------------------------------
jacobi_rotation ROT (
    .clk(clk), .rst(rst),
    .start(start),
    .col_i(col_i_sel), .col_j(col_j_sel),
    .col_i_rot(col_i_rot), .col_j_rot(col_j_rot),
    .done(rotation_done),
    .gamma_out(),
    .c_q16_out(c_q16_rot),   // fed to V_ROT
    .s_q16_out(s_q16_rot)
);

//------------------------------------
// Diagnostics
//------------------------------------
always_ff @(posedge clk) begin
    if (rotation_done) begin
        $display("ROT%0d pair=(%0d,%0d) |g01|=%0d |g02|=%0d |g12|=%0d thr=%0d conv=%0b",
                  rotation_count, col_i_idx, col_j_idx,
                  abs_g01, abs_g02, abs_g12, conv_threshold, all_converged);
    end
end
always_ff @(posedge clk)
begin
    if(all_converged)
    begin
        $display("FINAL GAMMAS");
        $display("g01=%0d", gamma_01);
        $display("g02=%0d", gamma_02);
        $display("g12=%0d", gamma_12);
        $display("thr=%0d", conv_threshold);
    end
end
endmodule