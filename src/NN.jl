mutable struct NN
    layers::Vector{Matrix}
end
struct Dense
    W::Matrix
    b::Matrix
    f::Function
end
function Dense(in::Integer, out::Integer, activation::Function)
    return Dense(rand(in, out), zeros(out), activation)
end
function (a::Dense)(x::AbstractArray)
    return a.f.(a.W*x .+ W.b)
end
function softmax(s)
end

