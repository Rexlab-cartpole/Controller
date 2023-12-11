import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

##

using VideoIO
using GLMakie

##

cam = VideoIO.opencamera()

##

img = read(cam)
obs_img = GLMakie.Observable(GLMakie.rotr90(img))
scene = GLMakie.Scene(camera=GLMakie.campixel!, resolution=reverse(size(img)))

GLMakie.image!(scene, obs_img)
display(scene)
fps = VideoIO.framerate(cam)
##

frames = 300
framecounter = 0

##

cam = VideoIO.opencamera("HD")

img = read(cam)
scene = Makie.Scene(resolution = size(img'))
# makieimg = Makie.image!(scene, img, show_axis = false, scale_plot = false)[end]
makieimg = Makie.image!(scene, img, show_axis = false, scale_plot = false)
Makie.rotate!(scene, -0.5pi)
display(scene)

fps = VideoIO.framerate(cam)

while isopen(scene)
    read!(cam, img)
    makieimg[1] = img
    sleep(1/fps)
end

close(cam)