module jacobi_params #(
    parameter N     = 3,
    parameter WIDTH = 32
)(
    input  logic signed [WIDTH-1:0] col_i [0:N-1],
    input  logic signed [WIDTH-1:0] col_j [0:N-1],
    // FIX (signedness): these now match how jacobi_rotation.sv declares them
    // (gamma in particular is a genuine cross-term and can be negative).
    output logic signed [2*WIDTH+$clog2(N)-1:0] alpha,
    output logic signed [2*WIDTH+$clog2(N)-1:0] beta,
    output logic signed [2*WIDTH+$clog2(N)-1:0] gamma
);

integer k;

// FIX (truncation): capture each product in an explicit 64-bit signed
// temporary via direct assignment. This gives the multiply a real
// context-determining target so it's computed at full width, instead of
// being silently sized to 32 bits as the self-determined left operand of
// the >>> shift (which is what was wrapping/overflowing before).
logic signed [63:0] prod_ii, prod_jj, prod_ij;

always_comb begin
    alpha = 0;
    beta  = 0;
    gamma = 0;
    for (k = 0; k < N; k = k + 1) begin
        prod_ii = col_i[k] * col_i[k];
        prod_jj = col_j[k] * col_j[k];
        prod_ij = col_i[k] * col_j[k];

        alpha = alpha + (prod_ii >>> 16);
        beta  = beta  + (prod_jj >>> 16);
        gamma = gamma + (prod_ij >>> 16);
    end
end

// NOTE: removed the $display that used to live here. It fired on every
// combinational re-evaluation of col_i/col_j (i.e. every time anything
// upstream glitched), which is noisy and not synchronized to an actual
// "this value was used for a rotation" event. Logging now happens in
// jacobi_rotation.sv, gated by the FSM, only when alpha/beta/gamma were
// actually latched and used.

endmodule