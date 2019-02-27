using Flux
export PolicyGradientState
mutable struct PolicyGradientState <: State
    discount_factor::Float64
    iteration::Integer  
    PolicyGradientState(;discount_factor::Float64=0.5) = new(discount_factor, 0)
end
function discount_rewards(rewards, v::Float64)
    return sum(r*(v^(i-1)) for (i, r) in enumerate(rewards))
end
function state_iteration(state::PolicyGradientState, car, world)

    reward = minimum(sense(car, w))
    push!(car.rewards, minimum(sensor_input))
    if car.crash
        reward = discount_rewards(car.rewards, car.discount_factor)
        advantage = rewards = mean(reward)
        car.model.train 
        car.inputs = []
        car.actions = []
        car.rewards = []
    end
 
end
function advance(carc::Car, state::PolicyGradientState, dt::Float64, w)
    car = Car.metadata
    sensor_input = sense(ccar, w)
    e = evaluate(car.nn, sensor_input)
    probs = e ./ sum(e)
    action = argmax(probs)
          
    if action == 1 # Forward
        car.vel = car.vel*1.1
    elseif action == 2
        car.vel = car.vel*0.9
    elseif action == 3
        car.vel = rotation_matrix(5)*car.vel
    elseif action == 4
        car.vel = rotation_matrix(-5)*car.vel
    end
    car.pos = car.pos + car.vel
    push!(car.inputs, sensor_input)
    action_v = zeros(4)
    action_v[action] = 1
    push!(car.actions, action_v)
end


struct PolicyGradientCar 
    model
    inputs::Vector
    actions::Vector
    rewards::Vector
    discount_factor::Float64
end

function PolicyGradientCar(discount_factor::Float64)
    model = Chain(
               Dense(7,100, σ),
               Dense(100,25, σ), 
               Dense(25,4, σ), 
               softmax)
    return PolicyGradientCar(
               model,
               [],
               [], 
               [],
               discount_factor)

end

function build_car(s::PolicyGradientState)
    return PolicyGradientCar(s.discount_factor)
end

