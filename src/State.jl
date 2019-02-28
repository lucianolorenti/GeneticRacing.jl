abstract type State end
function step_generation(state::State, world)
    for car in world.cars
        if !car.crash
            advance(state, car, 0.01, world)
            check_collision(car, world, state.iteration)
            state_iteration(state, car, world)
            if car_progress(car, world) > car.best_progress
                car.it_best_progress = state.iteration
                car.best_progress = car_progress(car, world)
            end
        end
    end
    state.iteration += 1
    generation_finished = all([car.crash for car in world.cars])
    if generation_finished
	state.iteration = 0
        create_new_generation(state, world)
    end
end
