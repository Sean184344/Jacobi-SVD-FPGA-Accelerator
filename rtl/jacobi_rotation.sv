module jacobi_rotation #(
    parameter N     = 3,
    parameter WIDTH = 32,
    parameter SCALE = 65536
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic signed [WIDTH-1:0] col_i [0:N-1],
    input  logic signed [WIDTH-1:0] col_j [0:N-1],
    output logic signed [31:0] col_i_rot [0:N-1],
    output logic signed [31:0] col_j_rot [0:N-1],
    output logic done,
    output logic signed [2*WIDTH+$clog2(N)-1:0] gamma_out,
    // NEW: expose cos/sin so jacobi_svd_3x3 can feed the same values into
    // the V-matrix column rotator without duplicating CORDIC logic.
    // Valid and stable for the full DONE cycle and beyond (until next start).
    output logic signed [31:0] c_q16_out,
    output logic signed [31:0] s_q16_out
);

//------------------------------------
// Parameter Generator
//------------------------------------
logic signed [2*WIDTH+$clog2(N)-1:0] alpha, beta, gamma;
jacobi_params #(.N(N), .WIDTH(WIDTH)) PARAMS (
    .col_i(col_i), .col_j(col_j),
    .alpha(alpha), .beta(beta), .gamma(gamma)
);
localparam int ACC_W = 2*WIDTH + $clog2(N);
logic signed [ACC_W+3:0] x_temp, y_temp;
assign x_temp = beta - alpha;
assign y_temp = gamma <<< 1;
//------------------------------------
// Dynamic normalization
// atan2(y,x) is scale-invariant so we can multiply both operands by any
// common factor. We choose the factor that puts the larger operand's MSB
// exactly at TARGET_MSB, keeping as many bits as possible for small
// differences while preventing overflow for large ones.
//------------------------------------
localparam int TARGET_MSB = 28;
function automatic int unsigned msb_pos(input logic [ACC_W+3:0] val);
    int unsigned pos;
    begin
        pos = 0;
        for (int i = ACC_W+3; i >= 0; i--) begin
            if (val[i]) begin pos = i; break; end
        end
        msb_pos = pos;
    end
endfunction
function automatic logic signed [31:0] normalize(
    input logic signed [ACC_W+3:0] val,
    input logic signed [7:0]       shift
);
    logic signed [ACC_W+3:0] tmp;
    begin
        if (shift >= 0) tmp = val >>> shift;
        else            tmp = val <<< (-shift);
        normalize = tmp[31:0];
    end
endfunction
logic [ACC_W+3:0]  abs_x_temp, abs_y_temp, norm_mag;
logic [7:0]        norm_msb_position;
logic signed [7:0] norm_shift;
assign abs_x_temp       = x_temp[ACC_W+3] ? (-x_temp) : x_temp;
assign abs_y_temp       = y_temp[ACC_W+3] ? (-y_temp) : y_temp;
assign norm_mag         = (abs_x_temp > abs_y_temp) ? abs_x_temp : abs_y_temp;
assign norm_msb_position = msb_pos(norm_mag);
// shift = msb_pos - TARGET: positive?right-shift; negative?left-shift
assign norm_shift = $signed({1'b0, norm_msb_position}) - TARGET_MSB;
//------------------------------------
// ATAN CORDIC
//------------------------------------
logic signed [31:0] x_atan_r, y_atan_r;
logic signed [ACC_W-1:0] gamma_r;
assign gamma_out = gamma_r;
logic [63:0] cordic_cartesian_data;
assign cordic_cartesian_data = {y_atan_r, x_atan_r};
logic atan_tvalid_r, angle_valid;
logic signed [31:0] angle_2theta;
cordic_atan_ip ATAN_IP (
    .aclk(clk),
    .s_axis_cartesian_tvalid(atan_tvalid_r),
    .s_axis_cartesian_tdata(cordic_cartesian_data),
    .m_axis_dout_tvalid(angle_valid),
    .m_axis_dout_tdata(angle_2theta)
);
//------------------------------------
// SINCOS CORDIC
//------------------------------------
logic signed [31:0] theta_r;
logic sincos_tvalid_r, sincos_valid;
logic [63:0] sincos_data;
cordic_sincos_ip SINCOS_IP (
    .aclk(clk),
    .s_axis_phase_tvalid(sincos_tvalid_r),
    .s_axis_phase_tdata(theta_r),
    .m_axis_dout_tvalid(sincos_valid),
    .m_axis_dout_tdata(sincos_data)
);
// IP packs output as {SIN, COS}
logic signed [31:0] c, s, c_q16, s_q16;
assign s     = sincos_data[63:32];
assign c     = sincos_data[31:0];
assign c_q16 = c >>> 14;
assign s_q16 = s >>> 14;
// Expose to top level for V-matrix rotation
assign c_q16_out = c_q16;
assign s_q16_out = s_q16;
//------------------------------------
// FSM
//------------------------------------
typedef enum logic [1:0] {IDLE, ATAN_WAIT, SINCOS_WAIT, DONE} state_t;
state_t state;
always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        state           <= IDLE;
        atan_tvalid_r   <= 1'b0;
        sincos_tvalid_r <= 1'b0;
        done            <= 1'b0;
        x_atan_r        <= '0;
        y_atan_r        <= '0;
        theta_r         <= '0;
        gamma_r         <= '0;
    end else begin
        done <= 1'b0;
        case (state)
            IDLE: begin
                if (start) begin
                    gamma_r <= gamma;
                    if (x_temp == 0 && y_temp == 0) begin
                        theta_r         <= 32'sd0;
                        sincos_tvalid_r <= 1'b1;
                        state           <= SINCOS_WAIT;
                        $display("DEGENERATE pair: alpha=%0d beta=%0d gamma=%0d -- identity rotation",
                                  alpha, beta, gamma);
                    end else begin
                        x_atan_r      <= normalize(x_temp, norm_shift);
                        y_atan_r      <= normalize(y_temp, norm_shift);
                        atan_tvalid_r <= 1'b1;
                        state         <= ATAN_WAIT;
                    end
                end
            end
            ATAN_WAIT: begin
                atan_tvalid_r <= 1'b0;
                if (angle_valid) begin
                    theta_r         <= angle_2theta >>> 1;
                    sincos_tvalid_r <= 1'b1;
                    state           <= SINCOS_WAIT;
                    $display("alpha=%0d beta=%0d gamma=%0d | x_atan=%0d y_atan=%0d | angle_2theta=%0d theta=%0d",
                              alpha, beta, gamma, x_atan_r, y_atan_r, angle_2theta, angle_2theta>>>1);
                end
            end
            SINCOS_WAIT: begin
                sincos_tvalid_r <= 1'b0;
                if (sincos_valid) begin
                    state <= DONE;
                    $display("c=%0d s=%0d  c_q16=%0d s_q16=%0d", c, s, c_q16, s_q16);
                end
            end
            DONE: begin
                done  <= 1'b1;
                state <= IDLE;
            end
        endcase
    end
end
//------------------------------------
// Column Rotator for A
//------------------------------------
column_rotator #(.N(N), .WIDTH(WIDTH), .SCALE(SCALE)) ROT (
    .c(c_q16), .s(s_q16),
    .col_i(col_i), .col_j(col_j),
    .col_i_rot(col_i_rot), .col_j_rot(col_j_rot)
);
endmodule