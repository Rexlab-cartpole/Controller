import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using LinearAlgebra
using ForwardDiff
using Plots
using Random 
using Printf
using MeshCat
using TrajOptPlots
using RobotZoo:Cartpole
using StaticArrays
using GeometryTypes
using ControlSystems
# using ColorTypes

using Plots

##

# Cartpole params
mc = 0.5
mp = 0.2
ℓ = 0.5
# b = 0.1
friction = 0

g = 9.81

h = 1/100

##

function cartpole_dynamics(x,u)
    r = x[1] # cart position
    θ = x[2] # pole angle
    ṙ = x[3] # change in cart position
    θ̇ = x[4] # change in pole angle
  
    F = u[1]
  
    a = [(m1+m2) m2*ℓ*cos(θ); m2*ℓ*cos(θ) J+m2*ℓ^2]
    b = [F - friction*ṙ + m2*ℓ*sin(θ)*θ̇ ^2; -m2*g*ℓ*sin(θ)]
  
    ẍ = a^(-1)*b
  
    return [ṙ; θ̇ ; ẍ[1]; ẍ[2]]
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
xg = [0; 0; 0; 0];

##

# Linearized state and control matrices
A = ForwardDiff.jacobian(dx->cartpole_rk4(dx, 0), xg)
B = ForwardDiff.derivative(du->cartpole_rk4(xg, du), 0)
display(A)
display(B)

##

nx = 4 # number of states
nu = 1 # number of controls

##

# Cost weights
# TODO: tune these!
Q = collect(Diagonal([1.0*ones(2); 1.0*ones(2)]));
R = 0.1;
Qn = Array(100.0*I(nx));

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
    u = -K*(x_lqr[k] - [0; 0; 0; 0])
    u_lqr[k] =  u[1]
    x_lqr[k+1] = A*x_lqr[k] + B*u_lqr[k]
end

plot(hcat(x_lqr...)', label=["x" "θ" "v" "θv"])
plot!(hcat(u_lqr...)', label="u")