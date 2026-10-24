module matrix_bank_3x3(
    input logic clk,
    input logic rst,
    input logic write_enable,
    input logic [1:0] col_i_idx,
    input logic [1:0] col_j_idx,
    input logic signed [31:0] col_i_new [0:2],
    input logic signed [31:0] col_j_new [0:2],
    input logic signed [31:0] matrix_init [0:2][0:2], // NEW: caller-supplied starting matrix
    output logic signed [31:0] col_i [0:2],
    output logic signed [31:0] col_j [0:2],
    output logic signed [31:0] matrix_out [0:2][0:2]  // NEW: full current matrix, independent of col_i_idx/col_j_idx
);

logic signed [31:0] matrix [0:2][0:2];

always_ff @(posedge clk or posedge rst)
begin
    if(rst)
    begin
        // NEW: load whatever the caller drove on matrix_init instead of a
        // fixed hardcoded matrix -- this is what lets the rest of the
        // design run on an arbitrary input matrix instead of needing a
        // recompile every time.
        for(int r=0; r<3; r=r+1)
        begin
            for(int cc=0; cc<3; cc=cc+1)
            begin
                matrix[r][cc] <= matrix_init[r][cc];
            end
        end
    end
    // FIX: both the debug-print loop AND the actual write loop are now
    // inside a single begin/end block gated by write_enable. Previously
    // the if(write_enable) only governed the $display for-loop -- the
    // actual matrix[][] write below it was a separate, unconditional
    // statement that ran every single clock cycle regardless of
    // write_enable, continuously clobbering the matrix with whatever
    // transient (often zero, not-yet-valid) value happened to be on
    // col_i_new/col_j_new at that instant.
    else if(write_enable)
    begin
        for(int wr=0; wr<3; wr=wr+1)
        begin
            $display(
                "WRITE pair=(%0d,%0d) row=%0d  old=(%0d,%0d) new=(%0d,%0d)",
                col_i_idx,
                col_j_idx,
                wr,
                matrix[wr][col_i_idx],
                matrix[wr][col_j_idx],
                col_i_new[wr],
                col_j_new[wr]
            );
        end

        for(int wr=0; wr<3; wr=wr+1)
        begin
            matrix[wr][col_i_idx] <= col_i_new[wr];
            matrix[wr][col_j_idx] <= col_j_new[wr];
        end
    end
end

always_comb
begin
    for(int r=0; r<3; r=r+1)
    begin
        col_i[r] = matrix[r][col_i_idx];
        col_j[r] = matrix[r][col_j_idx];
    end
end

// NEW: expose the full matrix at all times (not just the currently
// selected pair of columns) so the caller can read out the final result.
always_comb
begin
    for(int r=0; r<3; r=r+1)
    begin
        for(int cc=0; cc<3; cc=cc+1)
        begin
            matrix_out[r][cc] = matrix[r][cc];
        end
    end
end

// NOTE: removed the duplicate copy of this block -- it used to be
// written twice verbatim, which is why every real writeback printed
// "WRITEBACK" with identical matrix contents twice in a row.
always_ff @(posedge clk)
begin
    if(write_enable)
    begin
        $display("WRITEBACK");
        $display(
            "[%0d %0d %0d]",
            matrix[0][0],
            matrix[0][1],
            matrix[0][2]
        );
        $display(
            "[%0d %0d %0d]",
            matrix[1][0],
            matrix[1][1],
            matrix[1][2]
        );
        $display(
            "[%0d %0d %0d]",
            matrix[2][0],
            matrix[2][1],
            matrix[2][2]
        );
    end
end

endmodule