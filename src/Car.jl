include("Utils.jl")
mutable struct FFNN
    layers::Vector{Matrix}
end
function FFNN(layers_size::Vector{<:Integer})
    local layers = []
    for i=1:length(layers_size)-1
        push!(layers, randn(layers_size[i],layers_size[i+1]))
    end
    return FFNN(layers)
end
function sigmoid(x::Vector )
    return tanh.(x)
end
function number_of_layers(n::FFNN)
    return length(n.layers)
end
function evaluate(nn::FFNN, v::Vector)
    local layers = number_of_layers(nn)
    for j=1:layers-1
        v=sigmoid(nn.layers[j]'*v)
    end
	v = nn.layers[end]'*v
    return v
end
const car_width = 14
const car_height  = 24

mutable struct Car
    nn::FFNN
    pos::Vector
    dir::Vector
    crash::Bool
    fitness::Float64
    best_fitness::Float64
    it_best_fitness::Integer
    intersection::Vector # Debug information
    collision_point
end
function Car()
    return Car(FFNN(vec([13 13   3])), zeros(2), zeros(2), false, 0,-Inf,-1, [], nothing)
end
rotation_matrices = [rotation_matrix(ang) for ang=-90:15:90]
function sense(c::Car, w)
    local sensed_data = zeros(length(rotation_matrices))
    local lines_points = lines(w.track)
	local dir = c.dir

	c.intersection=[]
	for j=1:length(rotation_matrices)
		local new_dir =  rotation_matrices[j]*dir
		local p11= c.pos
   	    local p12= c.pos + new_dir*100
		local min_dist = Inf
		local min_intersection_point =nothing
		for l in lines_points
			(intersects, v1,v2) = intersection((p11,p12), l)
			if intersects
                intersection_point = p11+ (p12-p11)*v1
				dist = norm(p11-intersection_point)
				if dist < min_dist
					min_dist = dist
					min_intersection_point = intersection_point
				end
			end
		end

		if (min_intersection_point != nothing)
            sensed_data[j] = min_dist
			push!(c.intersection,min_intersection_point)
        else
            sensed_data[j] = 99999999
        end
	end
    return sensed_data
end
function fitness(c::Car, w)
    fitness(c.pos, w)
end
function fitness(pos::Vector, w)
    local min_dist = Inf
    local min_t = 0
    local min_j = 0
    local j = 0
    for j=1:length(w.track.points)-1
        local l = (w.track.points[j].pos, w.track.points[j+1].pos)
        local t = dot(pos-l[1], l[2]-l[1])/(norm(l[2]-l[1])^2)
        t = min(max(t,0),1)
        local nearest_point = l[1] + (l[2]-l[1])*t
        local dist = norm(pos-nearest_point)
        if dist < min_dist
            min_dist = dist
            min_t    = t
            min_j    = j
        end
    end
    return (min_t+min_j)/(length(w.track.points))
end
function check_collision(c::Car, w)
    local lines_points = lines(w.track)
    local α =  angle(c.dir)
    local w_2 = car_width/2
    local h_2 = car_height/2
    M=[-w_2 h_2; -w_2 -h_2; w_2 -h_2; w_2 h_2  ]

    rot = rotation_matrix(α)
    M=M*rot'
    M=M.+[c.pos[1] c.pos[2]]
    j = 1
    idx = [1 2 3 4 1]

    local t1 = nothing
    local t2 = nothing
    while j<=  length(lines_points) && !c.crash
        local l = lines_points[j]
        intersect = false
        local i=1
        while i<=4 && !c.crash
            (c.crash,t1,t2) =intersection(l,(M[idx[i],:],M[idx[i+1],:]))
            i=i+1
        end
        if !c.crash
          j=j+1
        end
    end
    if (c.crash)
        local l = lines_points[j]
        c.collision_point = l[1]+ (l[2]-l[1])*t1
    else
        c.collision_point = nothing
    end
	if !c.crash
		local inside_track = false
		for j=1:length(w.track.points)-1
			(P1, P2) = w.track.points[j].road_limits
			(P3, P4) = w.track.points[j+1].road_limits
			if (is_point_in_rectangle(P1, P2, P3, c.pos))
				inside_track = inside_track | true
				return
			end
		end
		if inside_track == false
			c.crash = true
		end
	end
end
function advance(car::Car, dt::Float64, w)
    local sensor_input = sense(car, w)
	local e  = evaluate(car.nn, sensor_input)
    car.pos = car.pos + dt*e[1:2]*10
	car.dir  = rotation_matrix(e[3])*car.dir

end
struct TrackPoint
    pos::Vector
    road_limits::Pair{Vector,Vector}
	dir::Vector
    width::Float64
end
function TrackPoint(pos::Vector, w::T) where T<:Number
    return TrackPoint(pos,Pair{Vector,Vector}(zeros(2),zeros(2)),zeros(2),w)
end

function update_road_limits_last_point(t::TrackPoint, p_prev::Vector)
    local p1 = t.pos
    dir            = p1-p_prev
    dir            = dir /norm(dir)
    v              = perpendicular(dir)
    t.road_limits = Pair{Vector,Vector}(p1 + v*t.width, p1 - v*t.width)
	t.dir  = dir
end
function update_road_limits(t::TrackPoint, p_next::Vector)
    local p1 = t.pos
    dir            = p_next-p1
    dir            = dir /norm(dir)
    v              = perpendicular(dir)
    t.road_limits = Pair{Vector,Vector}(p1 + v*t.width, p1 - v*t.width)
	t.dir  = dir
end

function update_road_limits(t::TrackPoint, p_prev::Vector, p_next::Vector)
    local pos  = t.pos
    local dir1 = pos-p_prev
    local dir2 = p_next-pos
    dir1=dir1/norm(dir1)
    dir2=dir2/norm(dir2)
    local v1 = perpendicular(dir1)
    local v2 = perpendicular(dir2)
    local v=(v1+v2)/2
    v=v/norm(v)
    t.road_limits= Pair{Vector,Vector}(pos+ v*t.width, pos - v*t.width)
	t.dir  = dir1
end
struct Track
    points::Vector{TrackPoint}
    lines::Vector
end
function Track()
    return Track([], [])
end
function lines(t::Track)
    if length(t.lines) == 0
        local l =[]
        for j=1:length(t.points)-1
            local p1 = t.points[j].road_limits
            local p2 = t.points[j+1].road_limits
            push!(l, (p1[1],p2[1]  ))
            push!(l, (p1[2],p2[2]  ))
        end
        return l
    end
    return t.lines
end
function perpendicular(v::Vector)
    local v1=similar(v)
    v1[2]=v[1]
    v1[1]=v[2]
    v1[2]=-v1[2]
    return v1/norm(v1)
end
function update_track!(t::Track)
    if length(t.points)<2
        return
    end
    update_road_limits(t.points[1],  t.points[2].pos)
    for j=2:length(t.points)-1
        update_road_limits(t.points[j], t.points[j-1].pos, t.points[j+1].pos)
    end
    update_road_limits_last_point(t.points[end], t.points[end-1].pos)

end
function find_points(t::Track, pos::Vector)
    if length(t.points) == 0
        return nothing
    end
    (dist,index) = findmin([sum((point.pos - pos).^2) for point in t.points])
    if dist<20
        return t.points[index]
    else
        return nothing
    end
end

struct World
    track::Track
    cars::Vector{Car}
end
function update_world!(w::World)
	update_track!(w.track)
	update_cars!(w)
end
function update_cars!(world::World)
	local v = world.track.points[1].dir
	for j=1:length(world.cars)
		world.cars[j].crash = false
		world.cars[j].intersection=[]
		world.cars[j].collision_point
		world.cars[j].dir = v
		world.cars[j].best_fitness = -Inf
		world.cars[j].it_best_fitness = -1
		world.cars[j].pos = world.track.points[1].pos + 5*world.cars[j].dir
	end
end
function World(;ncars::Integer=15)
    return World(Track(),Car[Car() for i=1:ncars])
end
