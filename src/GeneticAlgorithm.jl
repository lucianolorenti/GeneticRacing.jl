include("NN.jl")
abstract type CrossoverAlgorithm
end
mutable struct GeneticAlgorithmState <: State
    tournament_proportion::Float64
    crossover::CrossoverAlgorithm
    iteration::Integer
end
export SwapColumnsCrossover,
       LinearCombinationCrossover
struct SwapColumnsCrossover <: CrossoverAlgorithm
    probability::Float64
end
function state_iteration(state::GeneticAlgorithmState, car, world)
    car.fitness = fitness(car, world)
    if car.fitness > car.metadata.best_fitness
	car.metadata.best_fitness = car.fitness
	car.metadata.it_best_fitness = state.iteration
    end

end
function crossover(s::SwapColumnsCrossover, p1::Car, p2::Car)
    child1=deepcopy(p1)
    child2=deepcopy(p2)
    for j=1:length(p1.metadata.nn.layers)
	if rand() <= s.probability
	    (nrows,ncols) = size(p1.metadata.nn.layers[j])
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
struct LinearCombinationCrossover <: CrossoverAlgorithm
end
function crossover(s::LinearCombinationCrossover, p1::Car, p2::Car)
    child1=deepcopy(p1)
    child2=deepcopy(p2)
    for j=1:length(p1.nn.layers)
        v1 = rand()
        child1.nn.layers[j] = p1.nn.layers[j]*v1  + p1.nn.layers[j]*(1-v1)
        child2.nn.layers[j] = p1.nn.layers[j]*(1-v1)  + p1.nn.layers[j]*v1
    end
    return (child1, child2)
end



function GeneticAlgorithmState(tournament_proportion::Float64,
                               crossover::CrossoverAlgorithm)
    return GeneticAlgorithmState(tournament_proportion,
                                crossover,
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
        (child1, child2) = crossover(state.crossover, parent1, parent2)
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
       if cars[ind].metadata.fitness > best_fitness
           best_fitness = cars[ind].metadata.fitness
	   best_car = i
       end
   end
    return cars[best_car]
end
function mutate!(c::Car)
    for j=1:length(c.nn.layers)
	c.nn.layers[j] += randn(size(c.nn.layers[j]))*0.1
    end
end

struct GeneticCar
    nn::NN
    fitness::Float64
    best_fitness::Float64
    it_best_fitness::Integer
end
function GeneticCar()
    return GeneticCar(FFNN(),  0,-Inf,-1)
end
function build_car(s::GeneticAlgorithmState)
    return GeneticCar()
end
function fitness(c::Car, w)
    fitness(c.pos, w)
end
function fitness(pos::Vector, w)
    min_dist = Inf
    min_t = 0
    min_j = 0
    j = 0
    for j=1:length(w.track.points)-1
        l = (w.track.points[j].pos, w.track.points[j+1].pos)
        t = dot(pos-l[1], l[2]-l[1])/(norm(l[2]-l[1])^2)
        t = min(max(t,0),1)
        nearest_point = l[1] + (l[2]-l[1])*t
        dist = norm(pos-nearest_point)
        if dist < min_dist
            min_dist = dist
            min_t    = t
            min_j    = j
        end
    end
    return (min_t+min_j)/(length(w.track.points))
end

