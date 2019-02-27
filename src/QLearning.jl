using Flux
using Statistics
using LinearAlgebra
export PolicyGradientState
mutable struct PolicyGradientState <: State
    discount_factor::Float64
    iteration::Integer  
    PolicyGradientState(;discount_factor::Float64=0.5) = new(discount_factor, 0)
end
function discount_rewards(rewards, v::Float64)
    return sum(r*(v^(i-1)) for (i, r) in enumerate(rewards))
end
function state_iteration(state::PolicyGradientState, carc::Car, w)
    car = carc.metadata
    sensor_input = sense(carc, w) 
    reward = car_progress(carc, w) * 1/minimum(sensor_input)
    push!(car.rewards, minimum(sensor_input))
    if carc.crash
        reward = discount_rewards(car.rewards, car.discount_factor)
        advantage = rewards = mean(reward)
        Flux.train!(Flux.crossentropy, 
                    [a.*r for (a,r) in zip(car.actions, reward)], car.inputs)
        car.inputs = []
        car.actions = []
        car.rewards = []
    end
 
end
function advance(state::PolicyGradientState, carc::Car, dt::Float64, w)
    car = carc.metadata
    sensor_input = sense(carc, w)
    probs = car.model(sensor_input)
    action = argmax(probs)
          
    if action == 1 # Forward
        carc.vel = carc.vel*1.1
    elseif action == 2
        carc.vel = carc.vel*0.9
    elseif action == 3
        carc.vel = rotation_matrix(5)*carc.vel
    elseif action == 4
        carc.vel = rotation_matrix(-5)*carc.vel
    end
   carc.pos = carc.pos + carc.vel
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

function PolicyGradientCar(;discount_factor::Float64=0.5)
    model = Chain(
               Dense(13,100, σ),
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
    return PolicyGradientCar(discount_factor=s.discount_factor)
end

