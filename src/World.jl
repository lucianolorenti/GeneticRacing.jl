mutable struct World
    track::Track
    cars::Vector{Car}
end
function update_world!(w::World)
    update_track!(w.track)
    update_cars!(w)
end
function update_cars!(world::World)
    v = world.track.points[1].dir
    for j=1:length(world.cars)
	world.cars[j].crash = false
	world.cars[j].intersection=[]
	world.cars[j].collision_point
	world.cars[j].vel = v
	world.cars[j].pos = world.track.points[1].pos + 5*world.cars[j].vel
    end
end
function World(state;ncars::Integer=15)
    return World(Track(),Car[Car(build_car(state)) for i=1:ncars])
end
