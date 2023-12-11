import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using LinearAlgebra
using SparseArrays
using StaticArrays
using ForwardDiff
using NPZ

####################################
## helper functions
####################################

function dynamics_rk4(x, u, dt)
    #RK4 integration with zero-order hold on u
    f1 = dynamics(x, u)
    f2 = dynamics(x + 0.5*dt*f1, u)
    f3 = dynamics(x + 0.5*dt*f2, u)
    f4 = dynamics(x + dt*f3, u)
    return x + (dt/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

function dynamics(x, u)

    mc=0.4
    mp=0.307
    l=0.352
    g=9.81

    r = x[1] # cart position
    θ = x[2] # pole angle
    rd = x[3] # change in cart position
    θd = x[4] # change in pole angle
    F = u[1] # force applied to cart
    
    θdd = (g*sin(θ) + cos(θ) * ((-F - mp*l*(θd^2) * sin(θ))/(mc + mp))) / (l*(4/3 - (mp*(cos(θ)^2))/(mc + mp)))
    rdd = (F + mp*l*((θd^2)*sin(θ) - θdd*cos(θ))) / (mc + mp)
  
    return [rd; θd; rdd; θdd]
    
end

function concatenate_trajectories(X_list::VecOrMat{<:VecOrMat{<:AbstractVector}}, x_eq::AbstractVector;
    start_ind=1, end_ind=length(X_list[1])
)

    x_eq_list = [x_eq for i in 1:length(X_list)]
    X_mat_list = concatenate_trajectory.(X_list, x_eq_list; start_ind=start_ind, end_ind=end_ind)

    X_mat = reduce(hcat, X_mat_list)

    return X_mat

end

function concatenate_trajectory(X::VecOrMat{<:AbstractVector}, x_eq::AbstractVector;
    start_ind=1, end_ind=length(X)
)

    X_mat = reduce(hcat, X[start_ind:end_ind]) .- kron(ones(1, end_ind-start_ind+1), x_eq)

    return X_mat
end

function concatenate_trajectories(X_list::VecOrMat{<:VecOrMat{<:AbstractVector}}; start_ind=1,
    end_ind=length(X_list[1])
)

    X_mat_list = concatenate_trajectory.(X_list; start_ind=start_ind, end_ind=end_ind)

    X_mat = reduce(hcat, X_mat_list)

    return X_mat

end

function concatenate_trajectory(X::VecOrMat{<:AbstractVector}; start_ind=1,
    end_ind=length(X)
)

    X_mat = reduce(hcat, X[start_ind:end_ind])

    return X_mat
end

####################################
## import data
####################################

data_dir = joinpath(@__DIR__, "data")


data_file_name_1 = "cartpole_train_data_five_minutes_random_moves.npz"
full_data_file_path_1 = joinpath(data_dir, data_file_name_1)
data_1 = npzread(full_data_file_path_1)

X_est_teacher_1 = [data_1["est_states"][i, :] for i in 1:size(data_1["est_states"], 1)]
X_pred_teacher_1 = [data_1["est_states_predicted"][i, :] for i in 1:size(data_1["est_states_predicted"], 1)]
U_teacher_1 = [data_1["filtered_controls"][i, :] for i in 1:size(data_1["filtered_controls"], 1)-1]
Y_pixels_teacher_1 = [Vector{Float64}(vec(data_1["frames"][i, :, :, :])) for i in 1:size(data_1["frames"], 1)]




# data_file_name_2 = "cartpole_train_data_1_perturbations.npz"
# full_data_file_path_2 = joinpath(data_dir, data_file_name_2)
# data_2 = npzread(full_data_file_path_2)

# X_est_teacher_2 = [data_2["est_states"][i, :] for i in 1:size(data_2["est_states"], 1)]
# X_pred_teacher_2 = [data_2["est_states_predicted"][i, :] for i in 1:size(data_2["est_states_predicted"], 1)]
# U_teacher_2 = [data_2["filtered_controls"][i, :] for i in 1:size(data_2["filtered_controls"], 1)-1]
# Y_pixels_teacher_2 = [Vector{Int64}(vec(data_2["frames"][i, :, :, :])) for i in 1:size(data_2["frames"], 1)]




X_est_teacher = [X_est_teacher_1]
X_pred_teacher = [X_pred_teacher_1]
U_teacher = [U_teacher_1]
Y_pixels_teacher = [Y_pixels_teacher_1]


full_eq_frame_path = joinpath(@__DIR__, "data", "equilibrium_frame.npz")
eq_frame_data = npzread(full_eq_frame_path)
y_eq_data = eq_frame_data["eq_frame"]
y_eq_pixel = Vector{Int64}(vec(y_eq_data))

####################################
## Time Parameters
####################################

f = 30 # simulation frequency
dt = 1/f # time step

####################################
## define image properties
####################################

resx = size(Y_pixel_data, 3)
resy = size(Y_pixel_data, 2)

####################################
## make pendulum model
####################################

num_x = 4
num_u = 1
num_y = resx*resy

x_eq = zeros(4)
u_eq = zeros(1)

####################################
## calculate linearized model
####################################

A = ForwardDiff.jacobian(x -> dynamics_rk4(x, u_eq, dt), x_eq)
B = ForwardDiff.jacobian(u -> dynamics_rk4(x_eq, u, dt), u_eq)

##################################################################
## learn L and C*L (inf-horizon Kalman gain and C*Kalman gain)
##################################################################

println("Creating LLS problem...")

RHS = concatenate_trajectories(X_est_teacher .- X_pred_teacher; start_ind=2)
X_mat_1 = concatenate_trajectories(-X_pred_teacher, -x_eq; start_ind=2)
X_mat_2 = concatenate_trajectories(Y_pixels_teacher; start_ind=2)

X_mat = vcat(X_mat_1[1:end÷2, :], X_mat_2, ones(1, size(X_mat_1, 2)))

println("Solving LLS problem...")

LHS = sparse(RHS * pinv(Matrix(X_mat)))

println("Extracting matrices...")

LC = Matrix(hcat(LHS[:, 1:num_x÷2], zeros(num_x, num_x÷2)))
L = LHS[:, num_x÷2+1:end-1]
d2 = Vector(LHS[:, end])

@show maximum(abs.(LHS * X_mat - RHS))

#######################################################
## save learned stuff
#######################################################

# save models
data_write_file = "learned_KF_matrices_3.npy"

npzwrite(joinpath(data_dir, data_write_file), Dict("L" => L, "LC" => LC, "d2" => d2))