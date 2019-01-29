module Reinforcement
export  GeneticAlgorithmState,
    World,
    GUIState,
    create_window

using Gtk.ShortNames, Graphics
include("Car.jl")
include("GeneticAlgorithm.jl")
struct GUIState
    world::World
    is_dragging::Bool
    current_vertex
    widgets::Dict{String,Any}
    timer
    ga_state::GeneticAlgorithmState
	debug::Bool
end
function GUIState(ga_state::GeneticAlgorithmState, world::World)

    return GUIState(world, false,nothing, Dict{String,Any}(), nothing, ga_state, false)
end
function update_canvas(s::GUIState)
    local c = s.widgets["canvas"]
    draw(c)
end
function label_width_changed(widget, state)
    try
        str = getproperty(widget,:text,String)
        if (state.current_vertex != nothing)
            local num = parse(str)
            if (num==nothing)
                num=5
            end
            state.current_vertex.width = num

            update_world!(state.world)
            update_canvas(state)
        end
    catch e
        println(e)
    end
end
function start_race(w,state)
    state.timer = Timer(timer->advance(state, timer), 0.001, 0.01)
end
function stop_race(w,state)
    close(state.timer)
end
function advance( state::GUIState, timer)
    step_generation(state.ga_state, state.world)
	update_canvas(state)
end
function create_toolbar(state)

    hbox = Box(:h)
    label_width = Entry()  # a widget for entering text
    signal_connect((w)->label_width_changed(w,state), label_width, "changed")
    setproperty!(label_width, :text, "")

    state.widgets["label_width"] = label_width
    g = Grid()
    g[1,1] = Label("Vertex Width")
    g[2,1] = label_width
    push!(hbox,g)
    btn = Button("Empezar")
    btn_stop = Button("Parar")
    signal_connect((w)->start_race(w,state), btn, "clicked")
    signal_connect((w)->stop_race(w,state), btn_stop, "clicked")
    push!(hbox,btn)
    push!(hbox, btn_stop)
    return hbox
end

function canvas_button2press(widget, event, state)
    ctx = getgc(widget)
    local pos = [event.x;event.y]
    local t = state.world.track
    point = find_points(t, pos)
    if point != nothing
		filter!(e->e!=point, state.world.track.points)
    end

    state.current_vertex = nothing
    update_world!(state.world)
    update_canvas(state)
end
function canvas_buttonpress(widget, event, state)
    ctx = getgc(widget)
    local pos = [event.x;event.y]
    local t = state.world.track
    point = find_points(t, pos)
    if point == nothing
        point = TrackPoint(pos,35)
        push!(t.points,point)
        state.is_dragging=false
    else
        state.is_dragging=true
    end

    state.current_vertex = point
    update_world!(state.world)
    update_canvas(state)
end
function canvas_mousemove(widget, event, state)
    if state.is_dragging
        state.current_vertex.pos=[event.x;event.y];
        update_world!(state.world)
    end
#=    if length(state.world.track.points) >3
        println(fitness(vec([event.x,event.y]), state.world))
    end=#
    update_canvas(state)
end
function canvas_button1release(widget, event, state)
    state.is_dragging=false
    if (state.current_vertex!=nothing)
        setproperty!(state.widgets["label_width"],:text,string(state.current_vertex.width))
    end
end
function create_canvas(state)
    local c = @Canvas(1600,1600)
    local s = ScrolledWindow()
    Gtk.GAccessor.size_request(s,700,600)
    push!(s,c)
    c.mouse.button1press = (w,e)->canvas_buttonpress(w,e,state)
    c.mouse.button1release = (w,e)->canvas_button1release(w,e,state)
    c.mouse.button1motion = (w,e)->canvas_mousemove(w,e,state)

    c.mouse.button2release = (w,e)->canvas_button2press(w,e,state)
    draw(canvas -> draw_scene(canvas, state), c)
    state.widgets["canvas"] = c
    return s
end
function clear_screen(canvas)
    local ctx = getgc(canvas)
    local h = height(canvas)
    local w = width(canvas)
    rectangle(ctx, 0, 0, w, h)
    set_source_rgb(ctx, 1, 1, 1)
    fill(ctx)
	stroke(ctx)
end
import Gtk.draw
function draw(canvas, t::Track, state::GUIState)
    local ctx = getgc(canvas)
    set_source_rgb(ctx, 0, 1, 0)
    for point in t.points
        arc(ctx, point.pos[1], point.pos[2], 5, 0, 2pi)
        stroke(ctx)
    end

    set_source_rgb(ctx,0.8,0.8,0.8);
    set_line_width(ctx, 1.0);
    for j=1:length(t.points)-1
        local p1 = t.points[j].road_limits[1]
        local p2 = t.points[j+1].road_limits[1]
        move_to(ctx, p1[1],p1[2]);
        line_to(ctx,  p2[1],p2[2]);
        stroke(ctx)
        p1 = t.points[j].road_limits[2]
        p2 = t.points[j+1].road_limits[2]
        move_to(ctx, p1[1],p1[2]);
        line_to(ctx,  p2[1],p2[2]);
        stroke(ctx);
    end
end
function draw(canvas, car::Car, state::GUIState)

    local ctx = getgc(canvas)
	save(ctx)
	set_source_rgb(ctx,0.8,0.8,0.8);
	set_line_width(ctx, 1.0);
	local pos = car.pos
	w=14
	h=24
	translate(ctx, pos[1],pos[2])
	rotate(ctx, angle(car.dir))
	rectangle(ctx,-w/2 ,-h/2, w,h)
	stroke(ctx)
	restore(ctx)
	save(ctx)
	set_source_rgb(ctx, 1,0, 0)
#=	for int in car.intersection
		arc(ctx, int[1], int[2], 5, 0, 2pi)
		stroke(ctx)
	end=#
	if car.collision_point != nothing
		local int = car.collision_point
		arc(ctx, int[1], int[2], 5, 0, 2pi)
		stroke(ctx)
	end
#=			for j=1:length(rotation_matrices)
		local new_dir =  rotation_matrices[j]*car.dir
		local p11= car.pos
		local p12= car.pos + new_dir*50
		move_to(ctx,p11[1],p11[2])
		line_to(ctx,p12[1],p12[2])
		stroke(ctx)
	end=#
	restore(ctx)
end
function draw_scene(canvas, state::GUIState)

    local ctx = getgc(canvas)
	clear_screen(canvas)
    save(ctx)
	draw(canvas, state.world.track, state)
    if length(world.track.points) > 0
        for car in world.cars
			draw(canvas, car,state)
        end
    end
    restore(ctx)
end
function create_window(state)
    win = Window( "Canvas")
    vbox = Box(:v)
    push!(vbox, create_canvas(state))
    push!(vbox, create_toolbar(state))
    push!(win, vbox)

    showall(win)
end
end
