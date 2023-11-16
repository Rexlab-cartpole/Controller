import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using LinearAlgebra
using ForwardDiff
using Plots
using Random 
using Printf
# using MeshCat
using TrajOptPlots
using RobotZoo:Cartpole
using StaticArrays
# using GeometryTypes
using ControlSystems
# using ColorTypes

using Plots

##

###### Cartpole params
#### Pole angle is measured as an offset from vertical (vertical = 0 radians) - clockwise is positive
#### Length is distance to the center of mass and is assumed to be half the length of the pole for moment of inertia calculation
mc = 0.5 # mass of the cart
mp = 0.2 # mass of the pole
ℓ = 0.5 # distance to the center of mass

g = 9.81

h = 1/100

##

function cartpole_dynamics(x,u)
    r = x[1] # cart position
    θ = x[2] # pole angle
    rd = x[3] # change in cart position
    θd = x[4] # change in pole angle
    F = u[1] # force applied to cart
    
    θdd = (g*sin(θ) + cos(θ) * ((-F - mp*ℓ*θd^2 * sin(θ))/(mc + mp))) / (ℓ*(4/3 - (mp*cos(θ)^2)/(mc + mp)))
    xdd = (F + mp*ℓ*(θd^2*sin(θ) - θdd*cos(θ))) / (mc + mp)
  
    return [rd; θd; θdd; xdd]
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
Q = collect(Diagonal([1; 1; 1; 1]));
R = 0.1;

##

# Might need to invert some of the gains depending on rotation / translation directions of the joints
K = dlqr(A,B,Q,R)

# K[1] = 0

##

control_lim = 5

Nsim = 500
x_lqr = [zeros(nx) for i = 1:Nsim]
x_lqr[1] = [0; .2; 0; 0]
u_lqr = zeros(Nsim-1)

for k = 1:Nsim-1
    u = -K*(x_lqr[k] - xg)
    u_lqr[k] = min(control_lim, max(u[1], -control_lim))
    x_lqr[k+1] = cartpole_rk4(x_lqr[k], u_lqr[k])
end

plot(hcat(x_lqr...)', label=["x" "θ" "v" "θv"])
plot!(hcat(u_lqr...)', label="u")