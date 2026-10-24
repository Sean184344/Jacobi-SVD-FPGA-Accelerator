`timescale 1ns/1ps

module tb_matrix_bank;

logic clk;
logic rst;

logic write_enable;

logic [1:0] col_i_idx;
logic [1:0] col_j_idx;

logic signed [31:0] col_i_new [0:2];
logic signed [31:0] col_j_new [0:2];

logic signed [31:0] col_i [0:2];
logic signed [31:0] col_j [0:2];

matrix_bank_3x3 DUT(
    .clk(clk),
    .rst(rst),
    .write_enable(write_enable),
    .col_i_idx(col_i_idx),
    .col_j_idx(col_j_idx),
    .col_i_new(col_i_new),
    .col_j_new(col_j_new),
    .col_i(col_i),
    .col_j(col_j)
);

always #5 clk = ~clk;

initial
begin

    clk = 0;

    rst = 1;
    write_enable = 0;

    #20;
    rst = 0;

    //--------------------------------
    // Read Pair (0,1)
    //--------------------------------

    col_i_idx = 0;
    col_j_idx = 1;

    #20;

    $display("Before Write");

    $display("col_i = %0d %0d %0d",
             col_i[0],col_i[1],col_i[2]);

    $display("col_j = %0d %0d %0d",
             col_j[0],col_j[1],col_j[2]);

    //--------------------------------
    // Pretend these came from Jacobi
    //--------------------------------

    col_i_new[0] = 10;
    col_i_new[1] = 20;
    col_i_new[2] = 30;

    col_j_new[0] = 40;
    col_j_new[1] = 50;
    col_j_new[2] = 60;

    write_enable = 1;

    #10;

    write_enable = 0;

    //--------------------------------
    // Read again
    //--------------------------------

    #20;

    $display("After Write");

    $display("col_i = %0d %0d %0d",
             col_i[0],col_i[1],col_i[2]);

    $display("col_j = %0d %0d %0d",
             col_j[0],col_j[1],col_j[2]);

    #20;
    $finish;

end

endmodule