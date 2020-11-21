# Utility command to dump/undump pose of objects in gazebo
#  written by Yosuke Matsusaka
#  distributed under MIT License
import sys
import rospy
import math
import argparse
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def main():
    parser = argparse.ArgumentParser(description='Undump object pose.')
    parser.add_argument('--base-frame', dest='base_frame',
                        type=str, default='world',
                        help='base frame (default: world)')
    parser.add_argument('--offset-x', dest='offset_x',
                        type=float, default=0,
                        help='offset along X axis (default: 0)')
    parser.add_argument('--offset-y', dest='offset_y',
                        type=float, default=0,
                        help='offset along Y axis (default: 0)')
    parser.add_argument('--offset-z', dest='offset_z',
                        type=float, default=0,
                        help='offset along Z axis (default: 0)')
    args = parser.parse_args()
    undump(args)

def undump(opts):
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    for line in sys.stdin:
        (name, x, y, z, r, p, ay) = line.split(',')
        (x, y, z) = map(float, [x, y, z])
        (r, p, ay) = map(math.radians, map(float, [r, p, ay]))
        x += opts.offset_x
        y += opts.offset_y
        z += opts.offset_z
        quaternion = transformations.quaternion_from_euler(r, p, ay)
        model_state = ModelState()
        model_state.model_name = name
        model_state.reference_frame = opts.base_frame
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        model_state.pose.orientation.x = quaternion[0]
        model_state.pose.orientation.y = quaternion[1]
        model_state.pose.orientation.z = quaternion[2]
        model_state.pose.orientation.w = quaternion[3]
        set_model_state(model_state)
