`timescale 1ns/1ps

module tb_jacobi_rotation;

parameter N = 3;
parameter WIDTH = 8;

logic clk;

logic signed [WIDTH-1:0] col_i [0:N-1];
logic signed [WIDTH-1:0] col_j [0:N-1];

logic signed [31:0] col_i_rot [0:N-1];
logic signed [31:0] col_j_rot [0:N-1];

integer k;

jacobi_rotation DUT (
    .clk(clk),
    .col_i(col_i),
    .col_j(col_j),
    .col_i_rot(col_i_rot),
    .col_j_rot(col_j_rot)
);

always #5 clk = ~clk;

initial clk = 0;

initial begin

    // Test columns
    col_i[0] = 100;
    col_i[1] = 200;
    col_i[2] = 300;

    col_j[0] = 400;
    col_j[1] = 500;
    col_j[2] = 600;

    // Wait for CORDIC pipeline
    #1000;

    $display("--------------------------------");
    $display("INPUT COLUMNS");
    $display("--------------------------------");

    $display("col_i = %0d %0d %0d",
             col_i[0], col_i[1], col_i[2]);

    $display("col_j = %0d %0d %0d",
             col_j[0], col_j[1], col_j[2]);

    $display("");

    $display("--------------------------------");
    $display("JACOBI PARAMETERS");
    $display("--------------------------------");

    $display("alpha = %0d", DUT.alpha);
    $display("beta  = %0d", DUT.beta);
    $display("gamma = %0d", DUT.gamma);

    $display("");

    $display("--------------------------------");
    $display("CORDIC OUTPUTS");
    $display("--------------------------------");

    $display("angle_valid  = %0d", DUT.angle_valid);
    $display("sincos_valid = %0d", DUT.sincos_valid);

    $display("c      = %0d", DUT.c);
    $display("s      = %0d", DUT.s);

    $display("c_q16  = %0d", DUT.c_q16);
    $display("s_q16  = %0d", DUT.s_q16);

    $display("");
    $display("--------------------------------");
    $display("ROTATED COLUMNS");
    $display("--------------------------------");

    for(k=0; k<N; k=k+1)
    begin
        $display(
            "Row %0d : %0d %0d",
            k,
            col_i_rot[k],
            col_j_rot[k]
        );
    end

    $display("--------------------------------");

    $finish;

end

endmodule