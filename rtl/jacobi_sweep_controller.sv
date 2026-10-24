module jacobi_sweep_controller #(
    parameter MAX_ROTATIONS = 54   // safety ceiling; greedy exit fires first
                                   // for well-conditioned matrices.
                                   // 54 = 6 "sweeps" worth of budget.
)(
    input  logic clk,
    input  logic rst,
    input  logic rotation_done,    // from jacobi_rotation
    input  logic       all_converged, // from greedy selector in jacobi_svd_3x3
    input  logic [1:0] best_i,
    input  logic [1:0] best_j,
    output logic [1:0] col_i,
    output logic [1:0] col_j,
    output logic [5:0] rotation_count,
    output logic       start,
    output logic       done
);

logic busy;

always_ff @(posedge clk or posedge rst)
begin
    if (rst) begin
        col_i          <= 2'd0;
        col_j          <= 2'd1;
        rotation_count <= 6'd0;
        busy           <= 1'b0;
        start          <= 1'b0;
        done           <= 1'b0;
    end
    else begin
        start <= 1'b0;

        if (!done) begin
            if (!busy) begin
                if (all_converged || (rotation_count == 6'(MAX_ROTATIONS))) begin
                    done <= 1'b1;
                    $display("=== SVD DONE: %0d rotations, converged=%0b ===",
                              rotation_count, all_converged);
                end
                else begin
                    col_i <= best_i;
                    col_j <= best_j;
                    start <= 1'b1;
                    busy  <= 1'b1;
                end
            end
            else if (rotation_done) begin
                rotation_count <= rotation_count + 6'd1;
                busy           <= 1'b0;
            end
        end
    end
    $display(
 "CTRL converged=%0b rot=%0d busy=%0b",
 all_converged,
 rotation_count,
 busy
);
end

endmodule
