type GeneticAlgorithmState
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
function step_generation(state::GeneticAlgorithmState, world)
    for car in world.cars
        if !car.crash
            advance(car,0.01,world)
            check_collision(car,world)
            car.fitness = fitness(car,world)
			if car.fitness > car.best_fitness
				car.best_fitness = car.fitness
				car.it_best_fitness = state.iteration
			end
        end
    end
    state.iteration+=1
    generation_finished = all([car.crash || (abs(car.it_best_fitness-state.iteration)>50)  for car in world.cars])
    if generation_finished
		state.iteration = 0
        create_new_generation(state,world)
    end
end
function create_new_generation(state::GeneticAlgorithmState, world)
	local sort_idx = sortperm([car.fitness for car in world.cars], rev=true)
	local new_cars=[]
	local n_cars = length(world.cars)
	local renewed = round(Integer,length(world.cars)/3)
	local tournament_size = round(Integer,length(world.cars)*state.tournament_proportion)
	for i=1:renewed
		local parent1 = tournament(world.cars, tournament_size)
		local parent2 = tournament(world.cars, tournament_size)
        (child1, child2) = crossover(state.crossover_probability, parent1, parent2)
        mutate!(child1)
		mutate!(child2)
        push!(new_cars, child1)
		push!(new_cars,child2 )
    end
	local missing_cars = length(world.cars) - length(new_cars)
	world.cars = vcat(new_cars, world.cars[sort_idx[1:missing_cars]])

	update_cars!(world)
end
function tournament(cars, k)
   local best_fitness = -Inf
   local best_car = 0
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
	local child1=deepcopy(p1)
	local child2=deepcopy(p2)
	for j=1:length(p1.nn.layers)
		if rand() <= crossover_probability
			(nrows,ncols) = size(p1.nn.layers[j])
            for k=1:rand(1:ncols)
    			col_to_swap = rand(1:ncols)
    			local v1 = p1.nn.layers[j][:,col_to_swap]
	    		local v2 = p2.nn.layers[j][:,col_to_swap]
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
