
mutable struct StampedeObj
    relaxedIK
    baseIK
    pos_goals
    quat_goals
    times
    solution_graph # holds the joint configurations at each node
    score_graph # entries at layer k contain the aggregate score from their best found predecessor in layer k - 1
    num_nodes_per_layer # number of nodes per layer
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
    num_nodes_per_layer = zeros(num_time_points)
    predecessor_graph = zeros(max_nodes, num_time_points)
    active_nodes = Array{Array{Int64, 1}, 1}()
    for i = 1:num_time_points
        push!(active_nodes, Array{Int64, 1}())
    end
    times = range(0.,duration,length=num_time_points)

    return StampedeObj(relaxedIK, baseIK, pos_goals, quat_goals, times, solution_graph, score_graph, num_nodes_per_layer, predecessor_graph, active_nodes)
end

function solve(pos_goals, quat_goals, times)

end
