`timescale 1ns/1ps

module tb_jacobi_sweep_controller;

logic clk;
logic rst;
logic rotation_done;

logic [1:0] col_i;
logic [1:0] col_j;

logic [3:0] sweep_count;
logic done;

jacobi_sweep_controller #(
    .MAX_SWEEPS(3)
)
DUT(
    .clk(clk),
    .rst(rst),
    .rotation_done(rotation_done),
    .col_i(col_i),
    .col_j(col_j),
    .sweep_count(sweep_count),
    .done(done)
);

always #5 clk = ~clk;

initial
begin
    clk = 0;
end

initial
begin

    rst = 1;
    rotation_done = 0;

    #20;
    rst = 0;

    //--------------------------------
    // Generate 9 completed rotations
    //--------------------------------

    repeat(9)
    begin

        #20;
        rotation_done = 1;

        #10;
        rotation_done = 0;

    end

    #100;
    $finish;

end

always @(posedge clk)
begin

    $display(
        "time=%0t sweep=%0d pair=(%0d,%0d) done=%0d",
        $time,
        sweep_count,
        col_i,
        col_j,
        done
    );

end

endmodule