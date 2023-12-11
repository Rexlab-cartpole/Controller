import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using NPZ
using Plots
using GLMakie
using Printf

##

y = npzread("data/cartpole_train_data_five_minutes_with_perturbations.npz")
frames = y["frames"]
measurements = y["measurements"]
est_states = y["est_states"]
est_states_damped = y["est_states_damped"]
est_states_predicted = y["est_states_predicted"]
filtered_controls = y["filtered_controls"]
LQR_control_output = y["LQR_control_output"]


# Controls
Plots.plot(hcat(LQR_control_output...)', label=["LQR control (computed by -K*x)"])
Plots.plot!(hcat(filtered_controls...)', label=["filtered control (applied to odrive)"])

# State estimate from KF using encoder measurements
Plots.plot(est_states, label=["x" "θ" "xv" "θv"])

# Encoder measurements
Plots.plot(measurements, label=["measured x" "measured θ"])

image(frames[280,:,:]')
# gpos = frames[280,:,:]
# npzwrite("data/goalposition.npz", gpos)
