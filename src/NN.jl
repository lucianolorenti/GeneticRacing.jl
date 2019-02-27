mutable struct NN
    layers::Vector{Matrix}
end
struct aDense
    W::Matrix
    b::Matrix
    f::Function
end
function aDense(in::Integer, out::Integer, activation::Function)
    return Dense(rand(in, out), zeros(out), activation)
end
function (a::aDense)(x::AbstractArray)
    return a.f.(a.W*x .+ W.b)
end
function asoftmax(s)
end

