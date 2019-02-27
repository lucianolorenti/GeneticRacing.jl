include("Utils.jl")
using LinearAlgebra
const car_width = 14
const car_height  = 24

mutable struct Car
    pos::Vector
    vel::Vector
    crash::Bool
    intersection::Vector # Debug information
    collision_point
    it_best_progress
    best_progress
    metadata
end
function Car(metadata)
    return Car(zeros(2), zeros(2), false, [], nothing, 0, 0, metadata)
end
rotation_matrices = [rotation_matrix(ang) for ang=-90:15:90]
function sense(c::Car, w)
    sensed_data = zeros(length(rotation_matrices))
    lines_points = lines(w.track)
    dir = c.vel / norm(c.vel)
    
    c.intersection=[]
    for j=1:length(rotation_matrices)
        new_dir =  rotation_matrices[j]*dir
	p11= c.pos
   	p12= c.pos + new_dir*100
	min_dist = Inf
	min_intersection_point =nothing
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
function check_collision(c::Car, w)
    lines_points = lines(w.track)
    α =  angle(c.vel / norm(c.vel))
    w_2 = car_width/2
    h_2 = car_height/2
    M=[-w_2 h_2; -w_2 -h_2; w_2 -h_2; w_2 h_2  ]

    rot = rotation_matrix(α)
    M=M*rot'
    M=M.+[c.pos[1] c.pos[2]]
    j = 1
    idx = [1 2 3 4 1]

    t1 = nothing
    t2 = nothing
    while j<=  length(lines_points) && !c.crash
        l = lines_points[j]
        intersect = false
        i=1
        while i<=4 && !c.crash
            (c.crash,t1,t2) =intersection(l,(M[idx[i],:],M[idx[i+1],:]))
            i=i+1
        end
        if !c.crash
          j=j+1
        end
    end
    if (c.crash)
        l = lines_points[j]
        c.collision_point = l[1]+ (l[2]-l[1])*t1
    else
        c.collision_point = nothing
    end
	if !c.crash
		inside_track = false
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
function car_progress(c::Car, w)
    car_progress(c.pos, w)
end
function car_progress(pos::Vector, w)
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


