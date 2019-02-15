mutable struct GeneticAlgorithmState
    tournament_proportion::Float64
    crossover_probability::Float64
    iteration::Integer
end
function GeneticAlgorithmState(tournament_proportion::Float64,
                               crossover_probability::Float64)
    return GeneticAlgorithmState(tournament_proportion,
                                crossover_probability,
                                0)
end
function create_new_generation(state::GeneticAlgorithmState, world)
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
function tournament(cars, k)
   best_fitness = -Inf
   best_car = 0
   for i=1:k
       ind = rand(1:length(cars))
       if cars[ind].fitness > best_fitness
           best_fitness = cars[ind].fitness
	   best_car = i
       end
   end
    return cars[best_car]
end
function crossover(crossover_probability::Float64, p1::Car, p2::Car)
    child1=deepcopy(p1)
    child2=deepcopy(p2)
    for j=1:length(p1.nn.layers)
	if rand() <= crossover_probability
	    (nrows,ncols) = size(p1.nn.layers[j])
            for k=1:rand(1:ncols)
    		col_to_swap = rand(1:ncols)
    		v1 = p1.nn.layers[j][:,col_to_swap]
	    	v2 = p2.nn.layers[j][:,col_to_swap]
		child1.nn.layers[j][:,col_to_swap] = v2
		child2.nn.layers[j][:,col_to_swap] = v1
            end
	end
    end
    return (child1, child2)
end
function mutate!(c::Car)
    for j=1:length(c.nn.layers)
	c.nn.layers[j] += randn(size(c.nn.layers[j]))*0.1
    end
end
