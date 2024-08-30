#!/usr/bin/env python3

# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys
import numpy as np
from time import sleep
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

"""
This script makes the end-effector perform pick, pour, and place tasks.
Note that this script may not work for every arm as it was designed for the wx250.
Make sure to adjust commanded joint positions and poses as necessary.

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250

Then change to this directory and type:

    python3 bartender.py
"""


def main():
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
    )

    servo = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]

    robot_startup()

    if (bot.arm.group_info.num_joints < 5):
        bot.get_node().logfatal(
            'This demo requires the robot to have at least 5 joints!'
        )
        robot_shutdown()
        sys.exit()

    Home=[-0.003,-1.33,0.68,-0.9,-1.6]

    gate0 = [-1.55, 0.93, -1.8, 0.8, 0]#ok
    gate1 = [-1.55, 0.15, -0.4, 0.3, 0]#ok
    gate2 = [-1.55, 0, -0.54, 1, 0]#ok
    gate3 = [-1.55, -0.375, 0, 0.7, 0]#ok
    gate4 = [-1.55, -0.65, 0.3, 0.6, 0]#ok
    gate5 = [-1.55, -1, 0.61, 0.4, 0]#ok
    gate6 = [-1.55, -1.2, 0.45, 1, 0]#ok

    middle = [-2.02, -1.49, 0.33, -0.19, -0.06]

    GATES = [gate0, gate1, gate2, gate3, gate4, gate5, gate6]

    left_look = [-0.003,-1.33,0.68,-0.9, 0.02] 
    center_look = [-0.003,-1.33,0.68,-0.9, -1.53]
    right_look = [-0.003,-1.33,0.68,-0.9, -3.12]

    head = [left_look, center_look, right_look]
    transcaction = [-1.55, -1, -0.54, 1, -0.75]

    # joints=[0,0,0,0,0]

    # bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    # bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    # bot.gripper.release()
    # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    # bot.gripper.grasp()
    # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    # bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    # bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    # bot.gripper.release()
    # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()


    # bot.arm.go_to_home_pose()
    # sleep(1)
    # bot.arm.set_single_joint_position(joint_name='elbow', position=0.32)
    # # sleep(0.5)
    # bot.arm.set_single_joint_position(joint_name='wrist_angle', position=-0.02)
    # # sleep(0.5)
    # bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=-1.6)


    print(bot.arm.set_joint_positions(head[2]))
    sleep(0.5)
    print(bot.arm.set_joint_positions(head[0]))
    sleep(0.5)
    print(bot.arm.set_joint_positions(head[1]))
    sleep(0.5)

    print(bot.arm.set_joint_positions(Home))
    # sleep(0.5)
    # bot.gripper.grasp()
    # sleep(1)
    # bot.gripper.release()

    # for i in range(7):
    #     print(bot.arm.set_joint_positions(GATES[6-i]))
    #     sleep(5)
    print(bot.arm.set_joint_positions(middle))
    # print(bot.arm.set_joint_positions(transcaction))
    # print(bot.arm.set_joint_positions(gate6))
    # input()
    # print(bot.arm.set_joint_positions(transcaction))

    # print(bot.arm.set_joint_positions(gate5))
    # input()
    print(bot.arm.set_joint_positions(transcaction))

    print(bot.arm.set_joint_positions(gate0))
    input()
    print(bot.arm.set_joint_positions(transcaction))

    print(bot.arm.set_joint_positions(gate2))
    input()
    print(bot.arm.set_joint_positions(transcaction))

    
    # print(bot.arm.set_joint_positions(transcaction))



    # print(bot.arm.set_joint_positions(GATES[0]))

    while True:
        a = int(input("Choose Mottor: "))
        b = float(input("RADIANS: "))

        bot.arm.set_single_joint_position(servo[a], b)

    sleep(1)
    print(bot.arm.set_joint_positions(transcaction))


    print(bot.arm.set_joint_positions(transcaction))
    print(bot.arm.set_joint_positions(middle))
    print(bot.arm.set_joint_positions(Home))


    bot.arm.go_to_sleep_pose()

    robot_shutdown()


if __name__ == '__main__':
    main()
    
