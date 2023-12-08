import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using RobotZoo:Cartpole
using TrajOptPlots
using ControlSystems
using LinearAlgebra
using ForwardDiff
using StaticArrays
using Random

using Plots
using Printf

# using MeshCat
# using GeometryTypes
# using ColorTypes

##

###### Cartpole params
#### Pole angle is measured as an offset from vertical (vertical = 0 radians) - clockwise is positive
#### Length is distance to the center of mass and is assumed to be half the length of the pole for moment of inertia calculation
mc = 0.4 # mass of the cart (kg)
mp = 0.307 # mass of the pole (kg)
ℓ = 0.352 # distance to the center of mass (meters)

g = 9.81

h = 1/100

##

function cartpole_dynamics(x,u)
    r = x[1] # cart position
    θ = x[2] # pole angle
    rd = x[3] # change in cart position
    θd = x[4] # change in pole angle
    F = u[1] # force applied to cart
    
    θdd = (g*sin(θ) + cos(θ) * ((-F - mp*ℓ*(θd^2) * sin(θ))/(mc + mp))) / (ℓ*(4/3 - (mp*(cos(θ)^2))/(mc + mp)))
    rdd = (F + mp*ℓ*((θd^2)*sin(θ) - θdd*cos(θ))) / (mc + mp)
  
    return [rd; θd; rdd; θdd]
end
function cartpole_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = cartpole_dynamics(x, u)
    f2 = cartpole_dynamics(x + 0.5*h*f1, u)
    f3 = cartpole_dynamics(x + 0.5*h*f2, u)
    f4 = cartpole_dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    return xn
end

##

# Goal state
xg = [0; 0; 0; 0]; # position, angle, linear vel, angular vel

# Linearized state and control matrices
A = ForwardDiff.jacobian(dx->cartpole_rk4(dx, 0), xg)
B = ForwardDiff.derivative(du->cartpole_rk4(xg, du), 0)
display(A)
display(B)

##

nz = 2 # number of measured states
nx = 4 # number of states
nu = 1 # number of controls

# Cost weights
# TODO: tune these! (cart position, pole angle, cart linear velocity, pole angular velocity)
Q = collect(Diagonal([20; 20; 1; 1]));
R = .5;

# Might need to invert some of the gains depending on rotation / translation directions of the joints
K = dlqr(A,B,Q,R)

clipboard(@sprintf "k_matrix = np.array([%.5f, %.5f, %.5f, %.5f])" K...)

##

linear_enc_cpr = 600
angular_enc_cpr = 600
linear_vel_factor = 0.95
angle_vel_factor = 0.95

# Quantize the real state based on what the sensors would read
function measure_state(x) # sensor state is position, angle
    x_meas = [0.0;0.0]
    x_meas[1] = round(Int, linear_enc_cpr * x[1])/linear_enc_cpr
    x_meas[2] = round(Int, angular_enc_cpr * x[2])/angular_enc_cpr
    return x_meas
end

function estimate_state_basic_filter(x_meas, prev_x_meas, running_est_linear_vel, running_est_angle_vel)
    new_est_linear_vel = linear_vel_factor * (x_meas[1] - prev_x_meas[1])/h + (1 - linear_vel_factor) * running_est_linear_vel
    new_est_angle_vel = angle_vel_factor * (x_meas[2] - prev_x_meas[2])/h + (1 - angle_vel_factor) * running_est_angle_vel
    return [x_meas; new_est_linear_vel; new_est_angle_vel] # output is (estimated) [position, angle, linear vel, angular vel]
end

# Observation matrix H (also commonly written as C)
H = [1 0 0 0; 0 1 0 0] # Only position and angle are observable
meas_noise_cov = diagm([1; 1]) # nz x nz (where nz = length of measurement vector - 2 in this case)
process_noise_cov = diagm([1; 1; 1; 1]) # nx x nx

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
    x_pred = A*x + B*control_input

    # Extrapolate uncertainty
    est_cov_pred = A*est_cov*A' + process_noise_cov

    return x, x_pred, est_cov_pred
end

function kf_update(x_meas, x_pred_prev, est_cov_prev)
    Kn = est_cov_prev*H' * inv(H*est_cov_prev*H' + meas_noise_cov)
    x = x_pred_prev + Kn * (x_meas - H*x_pred_prev)
    est_cov = (I - Kn*H) * est_cov_prev * (I - Kn*H)' + Kn*meas_noise_cov*Kn'
    return x, est_cov
end

function kf_predict(state, control, est_cov)
    x_pred = A*state + B*control
    est_cov_pred = A*est_cov*A' + process_noise_cov
    return x_pred, est_cov_pred
end

##

control_lim = 3

Nsim = 500
x_lqr = [zeros(nx) for i = 1:Nsim]
x_lqr[1] = [0; -.2; 0; 0]
u_lqr = zeros(Nsim-1)

# Variables for estimate_state_basic_filter
running_est_linear_vel = 0
running_est_angle_vel = 0
measurements_prev = zeros(nz)

# Variables for Kalman filter
est_cov = diagm([100; 100; 100; 100])
est_cov_prev = diagm([100; 100; 100; 100])
measurements = zeros(nz)
x_pred_prev = x_lqr[1]
x_ests = [zeros(nx) for i = 1:Nsim-1]
x_ests[1] = x_lqr[1]
x_est = x_pred_prev

use_kf = true

for k = 1:Nsim-1
    # Measure new state
    measurements = measure_state(x_lqr[k][1:2]) # Outputs length 2 vector of sensor measurements

    # Do state estimate
    if use_kf
        if k > 1
            # x_est, x_pred_prev, est_cov_prev = kalman_filter(u_lqr[k-1], measurements, x_pred_prev, est_cov_prev)
            x_est, est_cov = kf_update(measurements, x_pred_prev, est_cov_prev)
        end
    else
        x_est = estimate_state_basic_filter(measurements, measurements_prev, running_est_linear_vel, running_est_angle_vel) # Outputs length nx vector state estimate
        measurements_prev = measurements
        running_est_linear_vel = x_est[3]
        running_est_angle_vel = x_est[4]
    end

    if k > 1
        x_ests[k] = x_est
    end

    # Do control
    u = -K*(x_est - xg)

    x_pred_prev, est_cov_prev = kf_predict(x_est, u[1], est_cov)

    u_lqr[k] = min(control_lim, max(u[1], -control_lim))
    x_lqr[k+1] = cartpole_rk4(x_lqr[k], u_lqr[k])
    # x_lqr[k+1] = x_lqr[k]
end

# plot(hcat(x_lqr...)', label=["x" "θ" "xv" "θv"])
plot(hcat(x_ests...)', label=["x" "θ" "xv" "θv"])
plot!(hcat(u_lqr...)', label="u")