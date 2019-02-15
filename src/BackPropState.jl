export BackPropState
mutable struct BackPropState <: State
end
function create_new_generation(state::BackPropState, world)
    sort_idx = sortperm([car.fitness for car in world.cars], rev=true)
    new_cars=[]
    n_cars = length(world.cars)
    renewed = round(Integer,length(world.cars)/3)
    tournament_size = round(Integer,length(world.cars)*state.tournament_proportion)
    for i=1:renewed
	parent1 = tournament(world.cars, tournament_size)
	parent2 = tournament(world.cars, tournament_size)
        (child1, child2) = crossover(state.crossover_probability, parent1, parent2)
        mutate!(child1)
	mutate!(child2)
        push!(new_cars, child1)
	push!(new_cars, child2)
    end
    missing_cars = length(world.cars) - length(new_cars)
    world.cars = vcat(new_cars, world.cars[sort_idx[1:missing_cars]])
    update_cars!(world)
end

