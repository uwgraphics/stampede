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
    solution_graph # holds the joint configurations at each node
    score_graph # entries at layer k contain the aggregate score from their best found predecessor in layer k - 1
    num_nodes_in_layers # number of nodes in layer
    predecessor_graph # entries at layer k contain the row index of the best found predecessor in layer k - 1
    active_nodes # entries in list k indicate that they have a predecessor in layer k - 1
end


function StampedeObj(relaxedIK, baseIK, pos_goals, quat_goals, duration)
    num_time_points = length(pos_goals)
    max_nodes = 1000
    num_dof = length(relaxedIK.relaxedIK_vars.vars.init_state)

    solution_graph = zeros(max_nodes, num_time_points, num_dof)
    score_graph = 1000000000000000.0 * ones(max_nodes, num_time_points)
    score_graph[:,1] = zeros(max_nodes, 1)
    num_nodes_in_layers = zeros(Int64, num_time_points)
    predecessor_graph = zeros(max_nodes, num_time_points)
    active_nodes = Array{Array{Int64, 1}, 1}()
    for i = 1:num_time_points
        push!(active_nodes, Array{Int64, 1}())
    end
    times = range(0.,duration,length=num_time_points)

    return StampedeObj(relaxedIK, baseIK, num_time_points, num_dof, pos_goals, quat_goals, times, solution_graph, score_graph, num_nodes_in_layers, predecessor_graph, active_nodes)
end

function solve(stampede; num_of_first_layer_samples = 200, min_samples_per_layer = 50)
    initialize_first_layer(stampede, num_of_first_layer_samples, min_samples_per_layer)
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
        prev_state = get_bounded_rand_state(stampede.relaxedIK.relaxedIK_vars)
        add_solution_to_layer(stampede, p_goal1, q_goal1, prev_state, 1)
    end

    if stampede.num_nodes_in_layers[1] < min_samples_per_layer
        result = fill_in_layer(stampede, 1, min_samples_per_layer)
        return false
    end

    return true
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

function add_solution_to_layer(stampede, position_goal, quat_goal, prev_state, layer_idx; pos_tol = 0.0001, rot_tol = 0.0001)
    xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(stampede.baseIK, position_goal, quat_goal, prev_state = prev_state, pos_tol=0.0001, rot_tol=0.0001)
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

function fill_in_layer(stampede, layer_idx, min_samples_per_layer; pos_tol = 0.0001, rot_tol = 0.0001)
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
            result = add_solution_to_layer(stampede, p_goal1, q_goal1, prev_state, layer_idx, pos_tol=pos_tol, rot_tol=rot_tol)
            println(result)

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

function gallop_to_next_layer(stampede, layer_idx, min_samples_per_layer = 50)
end
