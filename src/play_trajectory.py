#! /usr/bin/env python

from Stampede.Utils.ros_utils import *
import rospy
import os
import numpy as np
import sys
import tf
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

rospy.init_node('play_trajectory')

js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
tf_pub = tf.TransformBroadcaster()

path_to_src = os.path.dirname(__file__)
print path_to_src
f = open(path_to_src + '/Stampede/Config/relaxedik_path')
path_to_relaxedik_src =  f.readline()
path_to_relaxedik_src = path_to_relaxedik_src + '/src'
f.close()

sys.path.append(path_to_relaxedik_src )

from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj

y = get_relaxedIK_yaml_obj(path_to_relaxedik_src)
if not y == None:
    urdf_file_name = y['urdf_file_name']
    fixed_frame = y['fixed_frame']
    joint_ordering = y['joint_ordering']
    starting_config = y['starting_config']
    joint_state_define_file_name = y['joint_state_define_func_file']
    joint_state_define_file = open(
        path_to_relaxedik_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
    func = joint_state_define_file.read()
    exec (func)

rospy.sleep(0.1)

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

init_pos = np.array([ float(first_line_arr[3]), float(first_line_arr[4]),  float(first_line_arr[5])])


lines = []
line = f.readline()
while not line == '':
    lines.append(line)
    line = f.readline()

times = []
states = []
goal_positions = []
goal_orientations = []


for i in xrange(len(lines)):
    line_arr = lines[i].split(';')
    times.append(float(line_arr[0]))

    state_arr = line_arr[1].split(',')
    state = []
    for j in xrange(len(state_arr)):
        state.append(float(state_arr[j]))
    states.append(state)

    pos_arr = line_arr[2].split(',')
    goal_pos = [ float(pos_arr[0]),float(pos_arr[1]), float(pos_arr[2])   ]
    goal_positions.append(np.array(goal_pos) + init_pos)

    quat_arr = line_arr[3].split(',')
    quat = [ float(quat_arr[0]),float(quat_arr[1]), float(quat_arr[2]), float(quat_arr[3])   ]
    goal_orientations.append(quat)


rate = rospy.Rate(50)
idx = 0
while not rospy.is_shutdown():
    tf_pub.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         'common_world',
                         fixed_frame)

    if idx >= len(states):
        rospy.sleep(2.0)
        idx = 0
    xopt = states[idx]
    js = joint_state_define(xopt)


    if js == None:
        js = JointState()
        js.name = joint_ordering
        for x in xopt:
            js.position.append(x)
    now = rospy.Time.now()
    js.header.stamp.secs = now.secs
    js.header.stamp.nsecs = now.nsecs
    js_pub.publish(js)

    draw_linestrip_in_rviz(marker_pub, 'common_world', goal_positions, [0.,0.2,1.0,0.6], width=0.01)
    draw_linestrip_in_rviz(marker_pub, 'common_world', goal_positions[:idx], [0.,0.8,.2,0.8], width=0.02, id=1001)

    print js

    idx += 1

    rate.sleep()