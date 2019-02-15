abstract type State end
function step_generation(state::State, world)
    for car in world.cars
        if !car.crash
            advance(car, 0.01, world)
            check_collision(car, world)
            car.fitness = fitness(car, world)
	    if car.fitness > car.best_fitness
		car.best_fitness = car.fitness
		car.it_best_fitness = state.iteration
	    end
        end
    end
    state.iteration += 1
    generation_finished = all([car.crash || (abs(car.it_best_fitness-state.iteration)>50)  for car in world.cars])
    if generation_finished
	state.iteration = 0
        create_new_generation(state,world)
    end
end
