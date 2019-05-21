#!/usr/bin/env julia
println("Julia JIT compilation...")

include("Stampede/Utils_Julia/relaxed_ik_utils.jl")
include("Stampede/Utils_Julia/ik_task.jl")
include("Stampede/stampede2.jl")
include("Stampede/Utils_Julia/stampede_utils.jl")
path_to_src = Base.source_dir()
relaxedik_path = get_relaxedik_path(path_to_src)
using RobotOS

init_node("stampede")

#task_name = "spirals"
task_name = get_param("input_motion_file")
scaling_factor = get_param("scaling_factor")
#scaling_factor = 0.8
pos_goals, quat_goals = get_ik_task(path_to_src, task_name, scaling_factor=scaling_factor)

include(relaxedik_path * "/RelaxedIK/relaxedIK.jl")
include(relaxedik_path * "/RelaxedIK/Utils_Julia/joint_utils.jl")
# loaded_robot_file = open(relaxedik_path * "/RelaxedIK/Config/loaded_robot")
# loaded_robot = readline(loaded_robot_file)
# close(loaded_robot_file)
loaded_robot = get_param("robot_info_file")
println(loaded_robot)
if loaded_robot == ""
    println("ERROR: loaded_robot must be set in the stampede.launch file!")
    exit(-1)
end

relaxedIK = get_standard(relaxedik_path, loaded_robot)
baseIK = get_base_ik(relaxedik_path, loaded_robot)

s = StampedeObj(relaxedIK, baseIK, pos_goals, quat_goals, 15.0)
traj = solve(s)

fp = path_to_src * "/Stampede/OutputMotions/last_trajectory.stampede"
f = open(fp, "w")

init_pos = relaxedIK.relaxedIK_vars.init_ee_positions[1]

write(f, "$loaded_robot, $task_name, $scaling_factor, $(init_pos[1]), $(init_pos[2]), $(init_pos[3])\n")

for i = 1:length(traj)
    time = s.times[i]
    write(f, "$time; ")
    x = traj[i]
    for j = 1:length(x)-1
        write(f, "$(x[j]), ")
    end
    write(f, "$(x[end]);")
    p = pos_goals[i]
    q = quat_goals[i]

    write(f, "$(p[1]), $(p[2]), $(p[3]);")
    write(f, "$(q[1]), $(q[2]), $(q[3]), $(q[4])\n")
end

close(f)
