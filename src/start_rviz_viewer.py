#! /usr/bin/env python

import os
import rospy
import roslaunch

rospy.init_node('start_rviz_viewer')


path_to_src = os.path.dirname(__file__)
print path_to_src
f = open(path_to_src + '/Stampede/Config/relaxedik_path')
path_to_relaxedik_src =  f.readline()
f.close()

rospy.sleep(0.1)

# this is what's gonna have to change based on trjaectory file...
trajectory_name = rospy.get_param('trajectory_name', default='')
if trajectory_name == '':
    fp = path_to_src + '/Stampede/OutputMotions/last_trajectory.stampede'
    f = open(fp, 'r')
    first_line = f.readline()
    first_line_arr = first_line.split(',')
    robot_info = first_line_arr[0]
else:
    fp = path_to_src + '/Stampede/OutputMotions/{}.stampede'.format(trajectory_name)
    f = open(fp, 'r')
    first_line = f.readline()
    first_line_arr = first_line.split(',')
    robot_info = first_line_arr[0]

rospy.set_param('info_file_name', robot_info)
rospy.set_param('/relaxedIK/loaded_info_file_name', robot_info)
rospy.set_param('/relaxedIK/info_file_loaded', True)
f = open(path_to_relaxedik_src + '/src/RelaxedIK/Config/loaded_robot', 'w')
f.write(robot_info)
f.close()

rospy.sleep(0.1)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_path = path_to_src + '/../launch/rviz_viewer.launch'
launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
launch.start()

rospy.spin()