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


function dynamics_rk4(model, x, u, dt)
    #RK4 integration with zero-order hold on u
    f1 = RobotDynamics.dynamics(model, x, u)
    f2 = RobotDynamics.dynamics(model, x + 0.5*dt*f1, u)
    f3 = RobotDynamics.dynamics(model, x + 0.5*dt*f2, u)
    f4 = RobotDynamics.dynamics(model, x + dt*f3, u)
    return x + (dt/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

# State to linearize about
x_lin = [pi; 0] # angle, angular velocity

# Linearized state transition and control matrices
A = ForwardDiff.jacobian(dx->dynamics_rk4(model, dx, 0, dt), x_lin)
B = ForwardDiff.derivative(du->dynamics_rk4(model, x_lin, du, dt), 0)
display(A)
display(B)

Q = Diagonal([1., 1.])
R = Diagonal([0.1])

K = dlqr(A,B,Q,R)
display(K)

Nsim = 500

x_init_rk4 = [pi-.3; 0]
x_lqr_rk4 = [zeros(2) for _ in 1:Nsim]
x_lqr_rk4[1] = x_init_rk4

x_init = [pi-.3; 0]
x_lqr = [zeros(2) for _ in 1:Nsim]
x_lqr[1] = x_init

u_lqr = [zeros(1) for _ in 1:Nsim-1]

for k = 1:Nsim-1
    u_lqr[k] = -K*(x_lqr_rk4[k] - x_lin)
    x_lqr_rk4[k+1] = dynamics_rk4(model, x_lqr_rk4[k], u_lqr[k], dt)
    # A = ForwardDiff.jacobian(dx->dynamics_rk4(model, dx, 0, dt), x_lqr[k])
    # x_lqr[k+1] = A*(x_lqr[k] - x_lin) + x_lin + vec(B)*u_lqr[k][1]
    dx = A*(x_lqr[k] - x_lin) + vec(B)*u_lqr[k][1]
    x_lqr[k+1] = dx + x_lin
end

# plot((hcat(x_lqr...)' .+ hcat(x_lin...))[:,1], label=["θ_rollout" "ω_rollout"], ylimit=(-5,5))
plot(hcat(x_lqr...)'[:,1], label=["θ_rollout" "ω_rollout"], ylimit=(-5,5))
plot!(hcat(x_lqr_rk4...)'[:,1], label=["θ_rk4" "ω_rk4"], ylimit=(-5,5))
# plot!(hcat(x_kf_est...)'[:,1][:,:], label=["θ_est" "ω_est"])
# plot!(hcat(measurements_all...)', label="θ_meas")
plot!(hcat(u_lqr...)', label="u")