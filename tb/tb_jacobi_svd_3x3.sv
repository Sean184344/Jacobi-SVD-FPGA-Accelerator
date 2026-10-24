`timescale 1ns/1ps

module tb_jacobi_svd_3x3;

logic clk, rst;
logic signed [31:0] matrix_in  [0:2][0:2];
logic signed [31:0] matrix_out [0:2][0:2];
logic signed [31:0] V_out      [0:2][0:2];
logic svd_done;

function automatic logic signed [31:0] to_fixed(real val);
    return $rtoi(val * 65536.0);
endfunction
function automatic real to_real(logic signed [31:0] val);
    return real'(val) / 65536.0;
endfunction

jacobi_svd_3x3 #(
    .MAX_ROTATIONS(108),
    .CONV_SHIFT(12)
) DUT (
    .clk(clk), .rst(rst),
    .matrix_in(matrix_in),
    .matrix_out(matrix_out),
    .V_out(V_out),
    .svd_done(svd_done)
);

always #5 clk = ~clk;

real U     [0:2][0:2];
real V_mat [0:2][0:2];
real sigma [0:2];
real A_pinv[0:2][0:2];
    real sigma_max, trunc_ratio, threshold;
        real g01_raw, g02_raw, g12_raw, norm0, norm1, norm2;
initial begin
    clk = 0;

    // ----------------------------------------------------------
    // EDIT HERE -- any 3x3 matrix, fractions fine
    // ----------------------------------------------------------
    matrix_in[0][0]=to_fixed(4); matrix_in[0][1]=to_fixed(1); matrix_in[0][2]=to_fixed(2);
    matrix_in[1][0]=to_fixed(0); matrix_in[1][1]=to_fixed(3); matrix_in[1][2]=to_fixed(1);
    matrix_in[2][0]=to_fixed(0); matrix_in[2][1]=to_fixed(0); matrix_in[2][2]=to_fixed(2);

    rst = 1; #20; rst = 0;
    wait (svd_done);
    #20;

    // ----------------------------------------------------------
    // Step 1: singular values = column norms of matrix_out
    // ----------------------------------------------------------
    for (int k = 0; k < 3; k++) begin
        sigma[k] = 0.0;
        for (int r = 0; r < 3; r++)
            sigma[k] += to_real(matrix_out[r][k]) ** 2;
        sigma[k] = $sqrt(sigma[k]);
    end
    // ----------------------------------------------------------
    // Step 2: truncation threshold based on the LARGEST sigma
    //   FIX: threshold must be relative to sigma_max, NOT sigma[0].
    //   sigma[0] is typically the smallest (near-zero for
    //   near-rank-deficient matrices). Using sigma[0]*0.001 as the
    //   threshold means the near-zero singular value PASSES the
    //   filter and 1/sigma[0] ~200 blows up the pseudoinverse.
    // ----------------------------------------------------------
    sigma_max   = sigma[0];
    if (sigma[1] > sigma_max) sigma_max = sigma[1];
    if (sigma[2] > sigma_max) sigma_max = sigma[2];
    trunc_ratio = 0.001; // keep singular values >= 0.1% of the largest
    threshold   = sigma_max * trunc_ratio;
    // ----------------------------------------------------------
    // Step 3: U = normalised columns of matrix_out
    //   FIX: columns whose sigma < threshold are ZEROED in U
    //   instead of dividing by a near-zero sigma (which amplifies
    //   fixed-point quantisation noise by 1/sigma and produces
    //   garbage U columns -- that was the source of u01 = -0.73).
    // ----------------------------------------------------------
    for (int k = 0; k < 3; k++)
        for (int r = 0; r < 3; r++)
            U[r][k] = (sigma[k] > threshold)
                      ? to_real(matrix_out[r][k]) / sigma[k]
                      : 0.0;
    // V directly from hardware
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            V_mat[r][c] = to_real(V_out[r][c]);
    // ----------------------------------------------------------
    // Step 4: pseudoinverse  A+ = V * Sigma^-1 * U^T
    //   Only include terms where sigma[k] > threshold (truncated SVD).
    // ----------------------------------------------------------
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) begin
            A_pinv[r][c] = 0.0;
            for (int k = 0; k < 3; k++)
                if (sigma[k] > threshold)
                    A_pinv[r][c] += V_mat[r][k] / sigma[k] * U[c][k];
        end
    // ----------------------------------------------------------
    // Raw gamma values: true orthogonality check on matrix_out
    // directly (not via U normalization -- avoids the near-zero-
    // sigma amplification issue entirely).
    // ----------------------------------------------------------
    norm0 = sigma[0]; norm1 = sigma[1]; norm2 = sigma[2];
    g01_raw = 0.0; g02_raw = 0.0; g12_raw = 0.0;
    for (int r = 0; r < 3; r++) begin
        g01_raw += to_real(matrix_out[r][0]) * to_real(matrix_out[r][1]);
        g02_raw += to_real(matrix_out[r][0]) * to_real(matrix_out[r][2]);
        g12_raw += to_real(matrix_out[r][1]) * to_real(matrix_out[r][2]);
    end
    // ----------------------------------------------------------
    // Print
    // ----------------------------------------------------------
    $display("================================================");
    $display("SINGULAR VALUES  (sigma_max = %.6f, threshold = %.6f)",
              sigma_max, threshold);
    for (int k = 0; k < 3; k++)
        $display("  sigma[%0d] = %.6f  %s",
                  k, sigma[k],
                  (sigma[k] <= threshold) ? "<-- below threshold, ZEROED in A+" : "");

    $display("================================================");
    $display("RAW COLUMN ORTHOGONALITY of matrix_out  (should all be ~0)");
    $display("  dot(col0,col1) = %e  |  relative = %e",
              g01_raw,
              (norm0*norm1 > 1e-9) ? g01_raw/(norm0*norm1) : 0.0);
    $display("  dot(col0,col2) = %e  |  relative = %e",
              g02_raw,
              (norm0*norm2 > 1e-9) ? g02_raw/(norm0*norm2) : 0.0);
    $display("  dot(col1,col2) = %e  |  relative = %e",
              g12_raw,
              (norm1*norm2 > 1e-9) ? g12_raw/(norm1*norm2) : 0.0);

    $display("================================================");
    $display("U MATRIX  (columns with sigma <= threshold are zeroed)");
    for (int r = 0; r < 3; r++)
        $display("  [%8.5f  %8.5f  %8.5f]", U[r][0], U[r][1], U[r][2]);

    $display("U ORTHOGONALITY  (only meaningful for non-zeroed columns)");
    $display("  u01 = %f  %s", U[0][0]*U[0][1]+U[1][0]*U[1][1]+U[2][0]*U[2][1],
             (sigma[0]<=threshold||sigma[1]<=threshold) ? "(N/A: col zeroed)" : "");
    $display("  u02 = %f  %s", U[0][0]*U[0][2]+U[1][0]*U[1][2]+U[2][0]*U[2][2],
             (sigma[0]<=threshold||sigma[2]<=threshold) ? "(N/A: col zeroed)" : "");
    $display("  u12 = %f  %s", U[0][1]*U[0][2]+U[1][1]*U[1][2]+U[2][1]*U[2][2],
             (sigma[1]<=threshold||sigma[2]<=threshold) ? "(N/A: col zeroed)" : "");

    $display("================================================");
    $display("V MATRIX  (right singular vectors)");
    for (int r = 0; r < 3; r++)
        $display("  [%8.5f  %8.5f  %8.5f]", V_mat[r][0], V_mat[r][1], V_mat[r][2]);

    $display("V ORTHOGONALITY  (should be ~0 regardless)");
    $display("  v01 = %e",
        V_mat[0][0]*V_mat[0][1]+V_mat[1][0]*V_mat[1][1]+V_mat[2][0]*V_mat[2][1]);
    $display("  v02 = %e",
        V_mat[0][0]*V_mat[0][2]+V_mat[1][0]*V_mat[1][2]+V_mat[2][0]*V_mat[2][2]);
    $display("  v12 = %e",
        V_mat[0][1]*V_mat[0][2]+V_mat[1][1]*V_mat[1][2]+V_mat[2][1]*V_mat[2][2]);

    $display("================================================");
    $display("PSEUDOINVERSE  A+  (truncated SVD, ratio=%.4f)", trunc_ratio);
    for (int r = 0; r < 3; r++)
        $display("  [%9.5f  %9.5f  %9.5f]", A_pinv[r][0], A_pinv[r][1], A_pinv[r][2]);

    $display("================================================");
    $display("VERIFICATION: A * A+  (should be ~identity for full-rank,");
    $display("              or ~projection matrix for rank-deficient A)");
    begin
        real M_in[0:2][0:2], prod[0:2][0:2];
        for (int r=0; r<3; r++)
            for (int c=0; c<3; c++)
                M_in[r][c] = to_real(matrix_in[r][c]);
        for (int r=0; r<3; r++)
            for (int c=0; c<3; c++) begin
                prod[r][c] = 0.0;
                for (int k=0; k<3; k++)
                    prod[r][c] += M_in[r][k] * A_pinv[k][c];
            end
        for (int r=0; r<3; r++)
            $display("  [%7.4f  %7.4f  %7.4f]", prod[r][0], prod[r][1], prod[r][2]);
    end
    $display("================================================");

    $finish;
end
endmodule