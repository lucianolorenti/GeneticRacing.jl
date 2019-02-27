mutable struct TrackPoint
    pos::Vector
    road_limits::Pair{Vector,Vector}
    dir::Vector
    width::Float64
end
function TrackPoint(pos::Vector, w::T) where T<:Number
    return TrackPoint(pos,Pair{Vector,Vector}(zeros(2),zeros(2)),zeros(2),w)
end

function update_road_limits_last_point(t::TrackPoint, p_prev::Vector)
    p1 = t.pos
    dir = p1-p_prev
    dir = dir /norm(dir)
    v = perpendicular(dir)
    t.road_limits = Pair{Vector,Vector}(p1 + v*t.width, p1 - v*t.width)
    t.dir = dir
end
function update_road_limits(t::TrackPoint, p_next::Vector)
    p1 = t.pos
    dir = p_next-p1
    dir = dir /norm(dir)
    v = perpendicular(dir)
    t.road_limits = Pair{Vector,Vector}(p1 + v*t.width, p1 - v*t.width)
    t.dir = dir
end

function update_road_limits(t::TrackPoint, p_prev::Vector, p_next::Vector)
    pos = t.pos
    dir1 = pos-p_prev
    dir2 = p_next-pos
    dir1=dir1/norm(dir1)
    dir2=dir2/norm(dir2)
    v1 = perpendicular(dir1)
    v2 = perpendicular(dir2)
    v=(v1+v2)/2
    v=v/norm(v)
    t.road_limits= Pair{Vector,Vector}(pos+ v*t.width, pos - v*t.width)
    t.dir  = dir1
end
mutable struct Track
    points::Vector{TrackPoint}
    lines::Vector
end
function Track()
    return Track([], [])
end
function lines(t::Track)
    if length(t.lines) == 0
        l =[]
        for j=1:length(t.points)-1
            p1 = t.points[j].road_limits
            p2 = t.points[j+1].road_limits
            push!(l, (p1[1],p2[1]  ))
            push!(l, (p1[2],p2[2]  ))
        end
        return l
    end
    return t.lines
end
function perpendicular(v::Vector)
    v1=similar(v)
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


