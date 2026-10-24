`timescale 1ns/1ps

module tb_jacobi_params;

parameter N = 3;
parameter WIDTH = 8;

logic [WIDTH-1:0] col_i [0:N-1];
logic [WIDTH-1:0] col_j [0:N-1];

logic [2*WIDTH+$clog2(N)-1:0] alpha;
logic [2*WIDTH+$clog2(N)-1:0] beta;
logic [2*WIDTH+$clog2(N)-1:0] gamma;

jacobi_params #(
    .N(N),
    .WIDTH(WIDTH)
)
DUT (
    .col_i(col_i),
    .col_j(col_j),
    .alpha(alpha),
    .beta(beta),
    .gamma(gamma)
);

initial begin

    col_i[0] = 1;
    col_i[1] = 2;
    col_i[2] = 3;

    col_j[0] = 4;
    col_j[1] = 5;
    col_j[2] = 6;

    #10;

    $display("alpha = %0d", alpha);
    $display("beta  = %0d", beta);
    $display("gamma = %0d", gamma);

    $finish;

end

endmodule