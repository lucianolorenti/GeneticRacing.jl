using Flux
using Statistics
using LinearAlgebra
using StatsBase
export PolicyGradientState
mutable struct PolicyGradientState <: State
    discount_factor::Float64
    iteration::Integer  
    PolicyGradientState(;discount_factor::Float64=0.5) = new(discount_factor, 0)
end
function discount_rewards(rewards, v::Float64)
    return sum(r*(v^(i-1)) for (i, r) in enumerate(rewards))
end
function create_new_generation(state::PolicyGradientState, world)
    #sort_idx = sortperm([car.best_progress for car in world.cars], rev=true)
    #new_cars=[]
    #n_cars = length(world.cars)
    #renewed = round(Integer,length(world.cars)/3)
    #for i=1:renewed
    #    push!(new_cars, child1)
    #	push!(new_cars, child2)
    #end
    #missing_cars = length(world.cars) - length(new_cars)
    #world.cars = vcat(new_cars, world.cars[sort_idx[1:missing_cars]])
    update_cars!(world)
end

function state_iteration(state::PolicyGradientState, carc::Car, w)
    car = carc.metadata
    sensor_input = sense(carc, w)
    progress =  (10000*car_progress(carc, w))
    distance = minimum(sensor_input) 
    reward = (progress * distance) / (progress + distance)
    if carc.crash 
        push!(car.rewards, -1)
    else
        push!(car.rewards, 1)
    end
        
    if carc.crash
        reward = discount_rewards(car.rewards, car.discount_factor)
        advantage = rewards = mean(reward)
        lossf(x, y) = Flux.crossentropy(car.model(x), y)
        
        y =  [a.*r for (a,r) in zip(car.actions, reward)]
        ps = Flux.params(car.model)
        Flux.train!(lossf,
                    ps,
                    zip(car.inputs,y),
                    Flux.ADAM())
                   
        car.inputs = []
        car.actions = []
        car.rewards = []
    end
 
end
function advance(state::PolicyGradientState, carc::Car, dt::Float64, w)
    car = carc.metadata
    sensor_input = sense(carc, w)
    probs = car.model(sensor_input)
    action = sample(1:length(probs), Weights(probs))
          
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


mutable struct PolicyGradientCar 
    model
    inputs::Vector
    actions::Vector
    rewards::Vector
    discount_factor::Float64
end

function PolicyGradientCar(;discount_factor::Float64=0.5)
    model = Chain(
               Dense(13,25, σ),
               Dense(25,25, σ), 
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

