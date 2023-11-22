function rotx(θ)
    s, c = sincos(θ)
    return [1 0 0; 0 c -s; 0 s c]
end
function create_cartpole!(vis)
    meshcat.setobject!(vis[:cart], meshcat.HyperRectangle(meshcat.Vec(-.25,-1.0,-.15), meshcat.Vec(0.5,2,0.3)))
    meshcat.setobject!(vis[:pole], meshcat.Cylinder(meshcat.Point(0,0,-.75), meshcat.Point(0,0,.75), 0.05))
    meshcat.setobject!(vis[:a], meshcat.HyperSphere(meshcat.Point(0,0,0.0),0.1))
    meshcat.setobject!(vis[:b], meshcat.HyperSphere(meshcat.Point(0,0,0.0),0.1))
end
function update_cartpole_transform!(vis,x)
    pole_o = 0.3
    px = x[1]
    θ = x[2]
    meshcat.settransform!(vis[:cart], meshcat.Translation([0,px,0.0]))
    p1 = [pole_o,px,0]
    p2 = p1 + 1.5*[0,sin(θ), -cos(θ)]
    meshcat.settransform!(vis[:a], meshcat.Translation(p1))
    meshcat.settransform!(vis[:b], meshcat.Translation(p2))
    meshcat.settransform!(vis[:pole], meshcat.Translation(0.5*(p1 + p2)) ∘ meshcat.LinearMap(rotx(θ))) 
end

function animate_cartpole(X, dt)
    vis = meshcat.Visualizer()
    create_cartpole!(vis)
    anim = meshcat.Animation(floor(Int,1/dt))
    for k = 1:length(X)
        meshcat.atframe(anim, k) do
            update_cartpole_transform!(vis,X[k])
        end
    end
    meshcat.setanimation!(vis, anim)
    return meshcat.render(vis)
end