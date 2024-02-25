import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using RobotZoo:Cartpole
using RobotZoo
using RobotDynamics
using TrajOptPlots
using ControlSystems
using LinearAlgebra
using ForwardDiff
using SparseArrays
using StaticArrays
using Random
using TrajOptPlots


using Plots
using Printf

##

model = RobotZoo.Pendulum(b=0.0)

num_x = RobotZoo.state_dim(model)
num_u = RobotZoo.control_dim(model)

dt = 1/100

# x_eq = [5*π, 0.]
# u_eq = [0.0]
# # x0 = [π - deg2rad(10), -deg2rad(5)]
# x0 = [π - deg2rad(5), 0]
# # x0 = [π, 0]

# @test dynamics_rk4(model, x_eq, u_eq, 0.1) ≈ x_eq

##

# Goal state
x_lin = [π; 0] # angle, angular velocity
ug = [0]

function dynamics_rk4(model, x, u, dt)
    #RK4 integration with zero-order hold on u
    f1 = RobotDynamics.dynamics(model, x, u)
    f2 = RobotDynamics.dynamics(model, x + 0.5*dt*f1, u)
    f3 = RobotDynamics.dynamics(model, x + 0.5*dt*f2, u)
    f4 = RobotDynamics.dynamics(model, x + dt*f3, u)
    return x + (dt/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

# Linearized state and control matrices
A = ForwardDiff.jacobian(dx->dynamics_rk4(model, dx, ug, dt), x_lin)
B = ForwardDiff.jacobian(du->dynamics_rk4(model, x_lin, du, dt), ug)
display(A)
display(B)

##

nz = 1 # number of measured states
nx = 2 # number of states
nu = 1 # number of controls

# Cost weights
# TODO: tune these! (cart position, pole angle, cart linear velocity, pole angular velocity)
tf = 10.
Nt = 10
dt = 0.01
Q = Diagonal([1., 1.])
R = Diagonal([0.1])


# Might need to invert some of the gains depending on rotation / translation directions of the joints
K = dlqr(A,B,Q,R)

##

linear_enc_cpr = 600
angular_enc_cpr = 600
linear_vel_factor = 0.95
angle_vel_factor = 0.95

# Quantize the real state based on what the sensors would read
function measure_state(x) # sensor state is angle
    x_meas = [0.0]
    # x_meas[1] = round(Int, linear_enc_cpr * x[1])/linear_enc_cpr
    # x_meas[2] = round(Int, angular_enc_cpr * x[2])/angular_enc_cpr
    x_meas[1] = x[1]
    return x_meas
end

# Observation matrix H (also commonly written as C)
H = [1 0] # Only angle is observable
meas_noise_cov = diagm([1]) # nz x nz (where nz = length of measurement vector - 2 in this case)
process_noise_cov = diagm([1; 1]) # nx x nx

function kalman_filter(control_input, x_meas, x_pred_prev, est_cov_prev)
    # x_meas: nz x 1 (2x1)
    # est_cov_prev: nx x nx (4x4)
    # x_pred_prev: nx x 1 (4x1)
    # H: nz x nx (2x4)
    # meas_noise_cov: nz x nz (2x2)
    # process_noise_cov: nx x nx (4x4)
    # A: nx x nx (4x4)
    # B: nx x nu (4x1)


    # Update step

    # Compute Kalman gain
    Kn = est_cov_prev*H' * inv(H*est_cov_prev*H' + meas_noise_cov)

    # Update estimate with measurement
    x = x_pred_prev + Kn * (x_meas - H*x_pred_prev)

    # Update estimate uncertainty
    est_cov = (I - Kn*H) * est_cov_prev * (I - Kn*H)' + Kn*meas_noise_cov*Kn'


    # Predict step

    # Extrapolate state
    x_pred = A*(x-x_lin) + vec(B)*control_input + x_lin
    # dx = A*(x-x_lin) + vec(B)*control_input
    # x_pred = dx + x

    # Extrapolate uncertainty
    # est_cov_pred = A*est_cov*A' + process_noise_cov
    est_cov_pred = est_cov

    return x, x_pred, est_cov_pred
end



control_lim = Inf

Nsim = 500
x_lqr = [zeros(nx) for i = 1:Nsim]
x_lqr_rk4 = [zeros(nx) for i = 1:Nsim]
x_kf_est = [zeros(nx) for i = 1:Nsim-1]
x_lqr[1] = x_lin + [0; 0]
x_lqr_rk4[1] = x_lqr[1]
x_kf_est[1] = x_lqr[1]
u_lqr = zeros(Nsim-1)

# Variables for estimate_state_basic_filter
running_est_linear_vel = 0
running_est_angle_vel = 0
measurements_prev = zeros(nz)

# Variables for Kalman filter
est_cov_prev = diagm([100; 100])
measurements = zeros(nz)
measurements_all = [zeros(nz) for i = 1:Nsim-1]
x_pred_prev = x_lqr[1]
x_est = x_pred_prev

use_kf = false

for k = 1:Nsim-1
    # Measure new state
    measurements = measure_state(x_lqr_rk4[k]) # Outputs length 2 vector of sensor measurements
    measurements_all[k] = measurements
    # println(measurements)
    # Do state estimate
    # if use_kf
    #     if k > 1
    #         x_est, x_pred_prev, est_cov_prev = kalman_filter(u_lqr[k-1], measurements, x_pred_prev, est_cov_prev)
    #     end
    # else
    #     x_est = x_lqr[k]
    # end

    if k > 1
        x_est, x_pred_prev, est_cov_prev = kalman_filter(u_lqr[k-1], measurements, x_pred_prev, est_cov_prev)
        x_kf_est[k] = x_est
    end
    x_est = x_lqr_rk4[k]


    # Do control
    u = -K*(x_est - x_lin)
    u_lqr[k] = min(control_lim, max(u[1], -control_lim))
    x_lqr_rk4[k+1] = dynamics_rk4(model, x_lqr_rk4[k], u_lqr[k], dt)
    # x_lqr[k+1] = dynamics_rk4(model, x_lqr[k], u_lqr[k], dt)

    # x_lqr[k+1] = A*x_lqr_rk4[k]# + B*[u_lqr[k]]
    x_lqr[k+1] = A*x_lqr[k]# + B*[u_lqr[k]]
    # x_lqr[k+1] = dynamics_rk4(model, x_lqr[k], 0, dt)
    # x_lqr[k+1] = A*(x_est-x_lin) + B*([u_lqr[k]] - ug) + x_lqr[k]
end

# plot(hcat(x_lqr...)'[:,1], label=["θ_rollout" "ω_rollout"])
plot(hcat(x_lqr_rk4...)'[:,1], label=["θ_rk4" "ω_rk4"])
plot!(hcat(x_kf_est...)'[:,1][:,:], label=["θ_est" "ω_est"])
# plot!(hcat(measurements_all...)', label="θ_meas")
plot!(hcat(u_lqr...)', label="u")