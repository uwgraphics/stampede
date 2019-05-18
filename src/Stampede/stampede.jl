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
    score_graph # entries at layer k contain the aggregate score from their best found predecessor in layer k - 1
    num_nodes_in_layers # number of nodes in layer
    predecessor_graph # entries at layer k contain the row index of the best found predecessor in layer k - 1
    active_nodes # entries in list k indicate that they have a predecessor in layer k - 1
end

function StampedeObj(relaxedIK, baseIK, pos_goals, quat_goals, duration; pos_tol = 0.001, rot_tol = 0.001)
    num_time_points = length(pos_goals)
    max_nodes = 1000
    num_dof = length(relaxedIK.relaxedIK_vars.vars.init_state)

    solution_graph = zeros(max_nodes, num_time_points, num_dof)
    score_graph = 1000000000000000.0 * ones(max_nodes, num_time_points)
    score_graph[:,1] = zeros(max_nodes, 1)
    num_nodes_in_layers = zeros(Int64, num_time_points)
    predecessor_graph = zeros(Int64, max_nodes, num_time_points)
    active_nodes = Array{Array{Int64, 1}, 1}()
    for i = 1:num_time_points
        push!(active_nodes, Array{Int64, 1}())
    end
    times = range(0.,duration,length=num_time_points)

    return StampedeObj(relaxedIK, baseIK, num_time_points, num_dof, pos_goals, quat_goals, times, pos_tol, rot_tol, solution_graph, score_graph, num_nodes_in_layers, predecessor_graph, active_nodes)
end

function solve(stampede; num_of_first_layer_samples = 500, min_samples_per_layer = 30, num_clusters = 20)
    first_layer_result = initialize_first_layer(stampede, num_of_first_layer_samples, min_samples_per_layer)
    if ! first_layer_result
        return false
    end

    for i = 2:stampede.num_time_points
        success = gallop_to_next_layer(stampede, i, num_clusters, min_samples_per_layer)
        num_nodes = stampede.num_nodes_in_layers[i]
        println("result layer $i: $success, number of nodes in layer $i: $num_nodes")
        if ! success
            return false
        end
    end
end

function initialize_first_layer(stampede, num_of_first_layer_samples, min_samples_per_layer)
    q = stampede.quat_goals[1]
    q_goal1 = [Quat(q[1], q[2], q[3], q[4])]
    p_goal1 = [stampede.pos_goals[1]]

    #=
    for i=1:num_of_first_layer_samples
        xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(stampede.baseIK, p_goal1, q_goal1, prev_state = get_bounded_rand_state(stampede.relaxedIK.relaxedIK_vars), pos_tol=0.0001, rot_tol=0.0001)
        if valid_sol
            in_collision = stampede.relaxedIK.relaxedIK_vars.in_collision(xopt)
            if ! (in_collision)
                println("valid solution: $i")
                stampede.solution_graph[stampede.num_nodes_in_layers[1]+1, 1,:] = xopt
                stampede.num_nodes_in_layers[1] += 1
            end
        end
    end
    =#

    for i = 1:num_of_first_layer_samples
        println("Initializing first layer $i")
        prev_state = get_bounded_rand_state(stampede.relaxedIK.relaxedIK_vars)
        add_solution_to_first_layer(stampede, prev_state, 1)
    end

    success = fill_in_layer(stampede, 1, min_samples_per_layer, first_layer=true)

    return success
    #=
    while stampede.num_nodes_in_layers[1] < min_samples_per_layer
        r = rand(1:stampede.num_nodes_in_layers[1])
        xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(stampede.baseIK, p_goal1, q_goal1, prev_state = 0.9*(stampede.solution_graph[r,1,:] +  (0.1 * rand(stampede.num_dof))), pos_tol=0.0001, rot_tol=0.0001)
        println(valid_sol)
        if valid_sol
            in_collision = stampede.relaxedIK.relaxedIK_vars.in_collision(xopt)
            if ! (in_collision)
                stampede.solution_graph[stampede.num_nodes_in_layers[1]+1, 1,:] = xopt
                stampede.num_nodes_in_layers[1] += 1
            end
        end
        # stampede.solution_graph[stampede.num_nodes_in_layers[1]+1, 1,:] = xopt
    end
    =#
end

function add_solution_to_first_layer(stampede, prev_state, layer_idx)
    q = stampede.quat_goals[layer_idx]
    q_goal1 = [Quat(q[1], q[2], q[3], q[4])]
    p_goal1 = [stampede.pos_goals[layer_idx]]

    xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(stampede.baseIK, p_goal1, q_goal1, prev_state = prev_state, pos_tol=stampede.pos_tol, rot_tol=stampede.rot_tol)
    if valid_sol
        in_collision = stampede.relaxedIK.relaxedIK_vars.in_collision(xopt)
        if ! (in_collision)
            stampede.solution_graph[stampede.num_nodes_in_layers[1]+1, layer_idx,:] = xopt
            stampede.num_nodes_in_layers[1] += 1
            return true
        end
    end
    return false
end

function add_solution_to_kth_layer(stampede, prev_state, layer_idx)
    q = stampede.quat_goals[layer_idx]
    q_goal1 = [Quat(q[1], q[2], q[3], q[4])]
    p_goal1 = [stampede.pos_goals[layer_idx]]

    xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(stampede.baseIK, p_goal1, q_goal1, prev_state = prev_state, pos_tol=stampede.pos_tol, rot_tol=stampede.rot_tol)
    if valid_sol
        in_collision = stampede.relaxedIK.relaxedIK_vars.in_collision(xopt)
        if ! (in_collision)
            # stampede.solution_graph[stampede.num_nodes_in_layers[1]+1, layer_idx,:] = xopt
            # stampede.num_nodes_in_layers[1] += 1
            preceding_node_found = find_best_preceding_node(stampede, layer_idx, xopt)
            if preceding_node_found
               return true
            else
               return false
            end
        end
    end

    return false
end

function fill_in_layer(stampede, layer_idx, min_samples_per_layer; first_layer = false)
    if stampede.num_nodes_in_layers[layer_idx] == 0
        return false
    else
        q = stampede.quat_goals[layer_idx]
        q_goal1 = [Quat(q[1], q[2], q[3], q[4])]
        p_goal1 = [stampede.pos_goals[layer_idx]]

        num_nodes_in_layer = stampede.num_nodes_in_layers[layer_idx]
        original_samples = get_solutions_in_layer(stampede, layer_idx)
        count_idx = 1
        num_tries = 1
        while stampede.num_nodes_in_layers[layer_idx] < min_samples_per_layer
            println("$count_idx, $num_nodes_in_layer")
            prev_state = 0.8*(stampede.solution_graph[count_idx, layer_idx,:] +  (0.2 * rand(stampede.num_dof)))
            # prev_state = get_bounded_rand_state(stampede.relaxedIK.relaxedIK_vars)
            if first_layer
                result = add_solution_to_first_layer(stampede, prev_state, layer_idx)
            else
                result = add_solution_to_kth_layer(stampede, prev_state, layer_idx)
            end

            count_idx += 1
            if count_idx > num_nodes_in_layer
                count_idx = 1
            end
            num_tries += 1

            if num_tries > 10000
                println("layer could not be successfully filled in!")
                return false
            end
        end

        return true

    end
end

function get_solutions_in_layer(stampede, layer_idx; trans=false)
    if trans
        return transpose(stampede.solution_graph[1:stampede.num_nodes_in_layers[layer_idx], layer_idx, :])
    else
        return stampede.solution_graph[1:stampede.num_nodes_in_layers[layer_idx], layer_idx, :]
    end
end

function cluster_layer(stampede, layer_idx, num_clusters)
    solution_mat = get_solutions_in_layer(stampede, layer_idx, trans=true)
    R = kmeans(solution_mat, num_clusters; maxiter=200)
    centers = R.centers
    return centers, assignments(R), counts(R)
end

function find_best_preceding_node(stampede, layer_idx, candidate_added_state)
    # must return true or false depending on whether a preceding node is found
    nodes_in_previous_layer = get_solutions_in_layer(stampede, layer_idx - 1) # k x n matrix, where k is number of nodes in layer and n is robot DOFs

    best_state_found = nothing
    best_state_found_idx = 0
    best_score = 10000000000000000000000.0
    num_solutions_in_previous_layer = stampede.num_nodes_in_layers[layer_idx - 1]
    # loop through layers
    for i = 1:num_solutions_in_previous_layer
        candidate_previous_state = nodes_in_previous_layer[i,:]
        candidate_previous_state_to_come_score = stampede.score_graph[i, layer_idx - 1]
        dis = euclidean(candidate_previous_state, candidate_added_state)
        if candidate_previous_state_to_come_score + dis < best_score
            if check_legal_velocity(candidate_previous_state, candidate_added_state, t = 2.0*(stampede.times[layer_idx] - stampede.times[layer_idx-1]) , joint_velocity_limits=stampede.relaxedIK.relaxedIK_vars.robot.velocity_limits)
                best_state_found = candidate_previous_state
                best_state_found_idx = i
                best_score = candidate_previous_state_to_come_score + dis
            end
        end
    end

    if (best_state_found == nothing)
        return false
    else
        new_node_idx = stampede.num_nodes_in_layers[layer_idx] + 1
        stampede.solution_graph[new_node_idx, layer_idx, :] = best_state_found
        stampede.predecessor_graph[new_node_idx, layer_idx] = best_state_found_idx
        stampede.score_graph[new_node_idx, layer_idx] = best_score
        stampede.num_nodes_in_layers[layer_idx] += 1
        return true
    end
end

function gallop_to_next_layer(stampede, layer_idx, num_clusters, min_samples_per_layer; num_samples_per_cluster = 2)
    centers, a, cnts = cluster_layer(stampede, layer_idx-1, num_clusters)
    # prev_layer_solution = get_solutions_in_layer(layer_idx - 1)
    for i = 1:num_clusters
        for j = 1:num_samples_per_cluster
            # num_nodes_in_cluster = cnts[i]
            # rand_state_from_cluster = prev_layer_solution[]
            # prev_state = 0.9*centers[:,i] + 0.1*rand(stampede.num_dof)
            flip = rand(1:10)
            if flip < 2
                prev_state = get_bounded_rand_state(stampede.relaxedIK.relaxedIK_vars)
            else
                prev_state = 0.9*centers[:,i] + 0.01*rand(stampede.num_dof)
            end
            result = add_solution_to_kth_layer(stampede, prev_state, layer_idx)
            # println("gallop result: $result")
        end
    end
    success = fill_in_layer(stampede, layer_idx, min_samples_per_layer)
    return success
end
