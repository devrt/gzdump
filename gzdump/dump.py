# Utility command to dump/undump pose of objects in gazebo
#  written by Yosuke Matsusaka
#  distributed under MIT License
import rospy
import math
import argparse
from tf import transformations
from gazebo_msgs.srv import GetWorldProperties, GetModelState

def main():
    parser = argparse.ArgumentParser(description='Dump object pose.')
    parser.add_argument('--base-frame', dest='base_frame',
                        type=str, default='world',
                        help='base frame (default: world)')
    args = parser.parse_args()
    dump(args)

def dump(opts):
    rospy.wait_for_service('/gazebo/get_world_properties')
    rospy.wait_for_service('/gazebo/get_model_state')
    world_propertiies = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    for name in world_propertiies().model_names:
        pose = model_state(name, opts.base_frame).pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        euler = map(math.degrees, transformations.euler_from_quaternion(quaternion))
        print(','.join(map(str,[
            name,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            euler[0],
            euler[1],
            euler[2]
        ])))
