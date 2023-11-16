import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

using RobotZoo:Cartpole
using RobotDynamics
using ForwardDiff
using LinearAlgebra
using StaticArrays
using SparseArrays
using ControlSystems

using Plots
using Printf

##

# Cartpole Dynamics
# TODO: measure mc mp and l
mc = 0.1 # mass of the cart in kg (10)
mp = 0.2 # mass of the pole (point mass at the end) in kg
l = 0.5  # half the length of the pole in meters
g = 9.81 # gravity m/s^2

a = Cartpole(mc, mp, l, g)
h = 1/100

function dynamics_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = RobotDynamics.dynamics(a, x, u)
    f2 = RobotDynamics.dynamics(a, x + 0.5*h*f1, u)
    f3 = RobotDynamics.dynamics(a, x + 0.5*h*f2, u)
    f4 = RobotDynamics.dynamics(a, x + h*f3, u)
    return x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

##

Nx = 4     # number of state
Nu = 1     # number of controls
Tfinal = 5.0 # final time
Nt = Int(Tfinal/h)+1    # number of time steps
thist = Array(range(0,h*(Nt-1), step=h));

##

# Goal state
xg = [0; pi/2; 0; 0];

##

# Linearized state and control matrices
A = ForwardDiff.jacobian(dx->dynamics_rk4(dx, 0), xg)
B = ForwardDiff.derivative(du->dynamics_rk4(xg, du), 0)
display(A)
display(B)

##

# Cost weights
# TODO: tune these!
Q = collect(Diagonal([1.0*ones(2); 1.0*ones(2)]));
R = 0.1;

##

# Might need to invert some of the gains depending on rotation / translation directions of the joints
K = dlqr(A,B,Q,R)

##

Nsim = 500
x_lqr = [zeros(nx) for i = 1:Nsim]
x_lqr[1] = [0; 1; 0; 0]
u_lqr = zeros(Nsim-1)

for k = 1:Nsim-1
    # u_lqr[k] = -cache.Kinf*(x_lqr[k] - [0; 1; 0; 0])
    u = -K*(x_lqr[k] - xg)
    u_lqr[k] =  u[1]
    x_lqr[k+1] = A*x_lqr[k] + B*u_lqr[k]
end

plot(hcat(x_lqr...)', label=["x" "θ" "v" "θv"])
plot!(hcat(u_lqr...)', label="u")