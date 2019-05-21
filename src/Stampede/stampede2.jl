include("Utils_Julia/stampede_utils.jl")
using Clustering
using Distances

mutable struct StampedeObj
    relaxedIK
    baseIK
    num_time_points
    num_dof
    pos_goals
    quat_goals
    times
    pos_tol
    rot_tol
    solution_graph # holds the joint configurations at each node
    min_vel_score_graph
    minmax_vel_score_graph
    num_nodes_in_layers # number of nodes in layer
    predecessor_graph # entries at layer k contain the row index of the best found predecessor in layer k - 1
    active_nodes # entries in list k indicate that they have a predecessor in layer k - 1
end

function StampedeObj(relaxedIK, baseIK, pos_goals, quat_goals, duration; pos_tol = 0.0001, rot_tol = 0.0001)
    num_time_points = length(pos_goals)
    max_nodes = 10000
    num_dof = length(relaxedIK.relaxedIK_vars.vars.init_state)

    solution_graph = zeros(max_nodes, num_time_points, num_dof)
    min_vel_score_graph = zeros(max_nodes, num_time_points)
    minmax_vel_score_graph = zeros(max_nodes, num_time_points)
    num_nodes_in_layers = zeros(Int64, num_time_points)
    predecessor_graph = zeros(Int64, max_nodes, num_time_points)
    active_nodes = Array{Array{Int64, 1}, 1}()
    for i = 1:num_time_points
        push!(active_nodes, Array{Int64, 1}())
    end
    times = range(0.,duration,length=num_time_points)

    return StampedeObj(relaxedIK, baseIK, num_time_points, num_dof, pos_goals, quat_goals, times, pos_tol, rot_tol, solution_graph, min_vel_score_graph, minmax_vel_score_graph, num_nodes_in_layers, predecessor_graph, active_nodes)
end

function solve(stampede; max_stay_idx = 100, neighborhood_rad = 4.)
    println("initializing layers...")
    populate_layer(stampede, 1, max_stay_idx=max_stay_idx)

    for i = 2:stampede.num_time_points
        valid = gallop_to_next_layer(stampede, i)
        if ! valid
            println("no solution found, returning best trajectory to timepoint $(i-1)")
            return get_best_min_vel_trajectory(stampede, end_layer = i - 2)
        end
    end

    return get_best_min_vel_trajectory(stampede)
end

function initialize_layers(stampede)
end

function get_solution(stampede, layer_idx; prev_state = nothing)
    q = stampede.quat_goals[layer_idx]
    q_goal1 = [Quat(q[1], q[2], q[3], q[4])]
    p_goal1 = [stampede.pos_goals[layer_idx]]

    if prev_state == nothing
        prev_state = get_bounded_rand_state(stampede.relaxedIK.relaxedIK_vars)
    end

    in_collision = nothing
    valid_sol = false

    xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(stampede.baseIK, p_goal1, q_goal1, prev_state = prev_state, pos_tol=stampede.pos_tol, rot_tol=stampede.rot_tol, max_iter = 40)
    if valid_sol
        in_collision = stampede.relaxedIK.relaxedIK_vars.in_collision(xopt)
        if ! (in_collision)
            # stampede.solution_graph[stampede.num_nodes_in_layers[1]+1, layer_idx,:] = xopt
            # stampede.num_nodes_in_layers[1] += 1
            return xopt, valid_sol, in_collision
        end
    end
    return nothing, valid_sol, in_collision
end

function add_solution(stampede, x, layer_idx)
    num_nodes_in_layer = stampede.num_nodes_in_layers[layer_idx]
    stampede.solution_graph[num_nodes_in_layer + 1, layer_idx, :] = x
    stampede.num_nodes_in_layers[layer_idx] += 1
end

function get_solutions_in_layer(stampede, layer_idx; trans=false)
    if trans
        s = stampede.solution_graph[1:stampede.num_nodes_in_layers[layer_idx], layer_idx, :]
        # dims = size(s)
        # ret = rand(dims[2], dims[1])
        # return transpose!(s, ret)
        return my_transpose(s)
        # return transpose(stampede.solution_graph[1:stampede.num_nodes_in_layers[layer_idx], layer_idx, :])
    else
        return stampede.solution_graph[1:stampede.num_nodes_in_layers[layer_idx], layer_idx, :]
    end
end

function compute_layer_var_score(stampede, layer_idx)
    solutions = get_solutions_in_layer(stampede, layer_idx)
    c = cov(solutions)
    br = broadcast(abs, c)
    return sum(br)
end

function get_number_of_layer_clusters(stampede, layer_idx; neighborhood_rad = 0.5)
    num_solutions = stampede.num_nodes_in_layers[layer_idx]
    if num_solutions <= stampede.num_dof
        return num_solutions
    else
        solutions = get_solutions_in_layer(stampede, layer_idx, trans = true)
        # println(size(solutions))
        clusters = dbscan(solutions, neighborhood_rad, min_neighbors = 1, min_cluster_size = 1)
        return length(clusters)
    end
end

function populate_layer(stampede, layer_idx; max_stay_idx = 6)
    prev_num_clusters = 1
    stay_idx = 1
    try_idx = 1

    #=
    for i = 1:300
        x = get_solution(s, layer_idx)
        if ! (x[1] == nothing)
            add_solution(s, x[1], layer_idx)
            # println(get_solutions_in_layer(s, layer))
            score = get_number_of_layer_clusters(s, layer_idx)
            # println(score)
            # push!(layer_scores, score)
        end
     end
     =#

    while stay_idx <= max_stay_idx && try_idx < 10000
        xopt, valid_sol, in_collision = get_solution(stampede, layer_idx)
        if ! (xopt == nothing)
            add_solution(stampede, xopt, layer_idx)
            curr_num_clusters = get_number_of_layer_clusters(stampede, layer_idx)
            if curr_num_clusters <= prev_num_clusters
                stay_idx += 1
            else
                stay_idx = 1
            end
            prev_num_clusters = curr_num_clusters
        end
        try_idx += 1
    end
end

function populate_all_layers(stampede; max_stay_idx = 6)
    for i = 1:stampede.num_time_points
        populate_layer(stampede, i, max_stay_idx = max_stay_idx)
        num_nodes = stampede.num_nodes_in_layers[i]
        println("populated layer $i, $num_nodes nodes in layer")
    end
end

function gallop_to_next_layer(stampede, layer_idx)
    prev_layer_num_solutions = stampede.num_nodes_in_layers[layer_idx - 1]
    if prev_layer_num_solutions == 0
        return false
    end
    prev_layer_solutions = get_solutions_in_layer(stampede, layer_idx - 1)

    for i = 1:prev_layer_num_solutions
        prev_sol = prev_layer_solutions[i,:]
        xopt, valid_sol, in_collision = get_solution(stampede, layer_idx, prev_state = prev_sol)
        if ! (xopt == nothing)
            legal = check_legal_velocity(prev_sol, xopt; t = (stampede.times[layer_idx] - stampede.times[layer_idx-1]), joint_velocity_limits=stampede.relaxedIK.relaxedIK_vars.robot.velocity_limits)
            if legal
                new_node_idx = stampede.num_nodes_in_layers[layer_idx] + 1
                stampede.predecessor_graph[new_node_idx, layer_idx] = i
                vel = Distances.euclidean(prev_sol, xopt)
                stampede.min_vel_score_graph[new_node_idx, layer_idx] = stampede.min_vel_score_graph[i, layer_idx-1] + vel
                if vel >= stampede.minmax_vel_score_graph[i, layer_idx-1]
                    stampede.minmax_vel_score_graph[new_node_idx, layer_idx] = vel
                else
                    stampede.minmax_vel_score_graph[new_node_idx, layer_idx] = stampede.minmax_vel_score_graph[i, layer_idx-1]
                end
                add_solution(stampede, xopt, layer_idx)
                println("added solution $new_node_idx to layer $layer_idx")
            end
        end
    end
    return true
end

function get_best_minmax_vel_trajectory(stampede; end_layer = 0)
    if end_layer == 0
        end_layer = stampede.num_time_points
    end

    traj = [ ]
    num_solutions = stampede.num_nodes_in_layers[end_layer]
    curr_idx = argmin(stampede.minmax_vel_score_graph[1:num_solutions, end_layer])
    push!(traj, stampede.solution_graph[curr_idx, end_layer, :])

    for i = end_layer:-1:2
        curr_idx = stampede.predecessor_graph[curr_idx, i]
        push!(traj, stampede.solution_graph[curr_idx, i-1, :])
    end
    return traj
end

function get_best_min_vel_trajectory(stampede; end_layer = 0)
    if end_layer == 0
        end_layer = stampede.num_time_points
    end

    traj = [ ]
    num_solutions = stampede.num_nodes_in_layers[end_layer]
    curr_idx = argmin(stampede.min_vel_score_graph[1:num_solutions, end_layer])
    push!(traj, stampede.solution_graph[curr_idx, end_layer, :])

    for i = end_layer:-1:2
        curr_idx = stampede.predecessor_graph[curr_idx, i]
        push!(traj, stampede.solution_graph[curr_idx, i-1, :])
    end
    return reverse(traj)
end

function my_transpose(T)
    dims = size(T)
    ret = zeros(dims[2], dims[1])
    LinearAlgebra.transpose!(ret, T)
    return ret
end
