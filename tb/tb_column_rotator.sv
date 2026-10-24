`timescale 1ns/1ps

module tb_column_rotator;

parameter N = 3;
parameter WIDTH = 8;
parameter SCALE = 1000;

logic signed [31:0] c;
logic signed [31:0] s;

logic signed [WIDTH-1:0] col_i [0:N-1];
logic signed [WIDTH-1:0] col_j [0:N-1];

logic signed [31:0] col_i_rot [0:N-1];
logic signed [31:0] col_j_rot [0:N-1];

integer k;

column_rotator DUT(
    .c(c),
    .s(s),
    .col_i(col_i),
    .col_j(col_j),
    .col_i_rot(col_i_rot),
    .col_j_rot(col_j_rot)
);

initial begin

    c = 1000;   // 1.0
    s = 500;    // 0.5

    col_i[0] = 1;
    col_i[1] = 2;
    col_i[2] = 3;

    col_j[0] = 4;
    col_j[1] = 5;
    col_j[2] = 6;

    #10;

    $display("Rotated Columns:");

    for(k=0;k<N;k=k+1)
    begin

        $display(
            "Row %0d : %0d %0d",
            k,
            col_i_rot[k],
            col_j_rot[k]
        );

    end

    $finish;

end

endmodule