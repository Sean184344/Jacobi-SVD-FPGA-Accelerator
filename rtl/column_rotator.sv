module column_rotator #(
    parameter N = 3,
    parameter WIDTH = 32,
    parameter SCALE = 65536
)(
    input logic signed [31:0] c,
    input logic signed [31:0] s,
    input logic signed [WIDTH-1:0] col_i [0:N-1],
    input logic signed [WIDTH-1:0] col_j [0:N-1],
    output logic signed [31:0] col_i_rot [0:N-1],
    output logic signed [31:0] col_j_rot [0:N-1]
);
integer k;
logic signed [63:0] temp_i;
logic signed [63:0] temp_j;
always_comb begin

    for(k=0; k<N; k=k+1)
    begin

        temp_i =
            c * col_i[k]
          - s * col_j[k];

        temp_j =
            s * col_i[k]
          + c * col_j[k];

        col_i_rot[k] = temp_i >>> 16;
        col_j_rot[k] = temp_j >>> 16;

        $display(
            "k=%0d IN=(%0d,%0d) OUT=(%0d,%0d)",
            k,
            col_i[k],
            col_j[k],
            col_i_rot[k],
            col_j_rot[k]
        );

    end

end

endmodule