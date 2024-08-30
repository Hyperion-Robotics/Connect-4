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

from interbotix_xs_msgs.srv import TorqueEnable
from rclpy.node import Node


"""
This script makes the end-effector perform pick, pour, and place tasks.
Note that this script may not work for every arm as it was designed for the wx250.
Make sure to adjust commanded joint positions and poses as necessary.

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250

Then change to this directory and type:

    python3 bartender.py
"""
class torqueClient(Node):
    

    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(TorqueEnable, 'wx250/torque_enable')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TorqueEnable.Request()

    def send_request(self, cmd):
        if cmd==1:
            torque=True
        elif cmd==0:
            torque=False
        else:
            print("Error. Aborting")
            return 0
        self.req.cmd_type='group'
        self.req.name='all'
        self.req.enable=torque
        return self.cli.call_async(self.req)

class ROBOTIC_ARM:
    def __init__(self) -> None:
        self.bot = InterbotixManipulatorXS(
                        robot_model='wx250',
                        group_name='arm',
                        gripper_name='gripper',
                    )
        
        robot_startup()

        if (self.bot.arm.group_info.num_joints < 5):
            self.bot.get_node().logfatal(
                'This demo requires the robot to have at least 5 joints!'
            )
            robot_shutdown()
            sys.exit()
        
        home=[-0.003,-1.33,0.68,-0.9,-1.6]
        middle = [-2.02, -1.49, 0.33, -0.19, -0.06]
        ingame = [-1.70, -1, -0.54, 1, -0.75]

        gate0 = [-1.70, 0.93, -1.8, 0.8, 0]#ok
        gate1 = [-1.70, 0.15, -0.4, 0.3, 0]#ok
        gate2 = [-1.70, 0, -0.54, 1, 0]#ok
        gate3 = [-1.70, -0.375, 0, 0.7, 0]#ok
        gate4 = [-1.70, -0.65, 0.3, 0.6, 0]#ok
        gate5 = [-1.70, -1, 0.61, 0.4, 0]#ok
        gate6 = [-1.70, -1.2, 0.45, 1, 0]#ok

        left_look = [-0.003,-1.33,0.68,-0.9, 0.02] 
        center_look = [-0.003,-1.33,0.68,-0.9, -1.53]
        right_look = [-0.003,-1.33,0.68,-0.9, -3.12]

        self.POSITIONS = {
            "home": home,
            "midle": middle,
            "ingame":ingame,

            "left_look":[-0.003,-1.33,0.68,-0.9, 0.02] ,
            "center_look": [-0.003,-1.33,0.68,-0.9, -1.53],
            "right_look": [-0.003,-1.33,0.68,-0.9, -3.12],

            "gate0": [-1.70, 0.93, -1.8, 0.8, 0],
            "gate1": [-1.70, 0.15, -0.4, 0.3, 0],
            "gate2": [-1.70, 0, -0.54, 1, 0],
            "gate3": [-1.70, -0.375, 0, 0.7, 0],
            "gate4": [-1.70, -0.65, 0.3, 0.6, 0],
            "gate5": [-1.70, -1, 0.61, 0.4, 0],
            "gate6": [-1.70, -1.2, 0.45, 1, 0]
        }
        self.GATES = [gate0, gate1, gate2, gate3, gate4, gate5, gate6]
        self.head = [left_look, center_look, right_look]

        self.last_moved_to = self.POSITIONS["home"]

        self.servos = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]

        self.in_game = False

    def move(self,id, pos):
        self.bot.arm.set_single_joint_position(joint_name=self.servos[id] , position=pos)

    def move_to_pos(self,pos):
        if(self.last_moved_to != self.POSITIONS[pos]):
            self.bot.arm.set_joint_positions(self.POSITIONS[pos])
            self.last_moved_to = self.POSITIONS[pos]
        else:
            print("I AM ALREADY AT THAT POSITION")

    
    def hibernation_mode(self): #the potition where the torque is closed so that the servos can rest alitle
        if self.last_moved_to == self.POSITIONS["home"]:
            self.bot.arm.go_to_sleep_pose()
            pass##torn off torque
        elif self.last_moved_to == self.POSITIONS["midle"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["home"])
            self.bot.arm.go_to_sleep_pose()
            pass##torn off torque
        elif self.last_moved_to == self.POSITIONS["ingame"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["midle"])
            self.bot.arm.set_joint_positions(self.POSITIONS["home"])
            self.bot.arm.go_to_sleep_pose()
            pass##torn off torque
        self.last_moved_to = self.POSITIONS["home"]

    def wake_up(self):
        if  self.last_moved_to == self.POSITIONS["home"]:
            self.move_to_pos("home")
            self.move_to_pos("right_look")
            sleep(0.4)
            self.move_to_pos("left_look")
            sleep(0.4)
            self.move_to_pos("center_look")
            sleep(0.1)
            self.last_moved_to = self.POSITIONS["home"]

    def start_game(self):
        if self.last_moved_to == self.POSITIONS["home"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["midle"])
            self.bot.arm.set_joint_positions(self.POSITIONS["ingame"])
        elif self.last_moved_to == self.POSITIONS["midle"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["ingame"])
        self.last_moved_to = self.POSITIONS["ingame"]
        self.in_game = True
        

    def play_gate(self,id:int):
        self.bot.arm.set_joint_positions(self.GATES[id])
        sleep(0.2)
        self.bot.arm.set_single_joint_position(joint_name=self.servos[0] , position=-1.53)
        self.last_moved_to = self.POSITIONS[f"gate{id}"]

    def end_game(self):
        self.move_to_pos("midle")
        self.move_to_pos("home")
        self.hibernation_mode()
        self.in_game = False
        sleep(0.3)

    def arm_is_playing(self):
        return self.in_game





# def main():

#     # joints=[0,0,0,0,0]

#     # bot.arm.set_ee_pose_components(x=0.3, z=0.2)
#     # bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
#     # bot.gripper.release()
#     # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
#     # bot.gripper.grasp()
#     # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
#     # bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
#     # bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
#     # bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
#     # bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
#     # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
#     # bot.gripper.release()
#     # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
#     # bot.arm.go_to_home_pose()
#     # bot.arm.go_to_sleep_pose()


#     # bot.arm.go_to_home_pose()
#     # sleep(1)
#     # bot.arm.set_single_joint_position(joint_name='elbow', position=0.32)
#     # # sleep(0.5)
#     # bot.arm.set_single_joint_position(joint_name='wrist_angle', position=-0.02)
#     # # sleep(0.5)
#     # bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=-1.6)



#     # sleep(0.5)
#     # bot.gripper.grasp()
#     # sleep(1)
#     # bot.gripper.release()

#     # for i in range(7):
#     #     print(bot.arm.set_joint_positions(GATES[6-i]))
#     #     sleep(5)
#     print(bot.arm.set_joint_positions(middle))
#     # print(bot.arm.set_joint_positions(transcaction))
#     # print(bot.arm.set_joint_positions(gate6))
#     # input()
#     # print(bot.arm.set_joint_positions(transcaction))

#     # print(bot.arm.set_joint_positions(gate5))
#     # input()
#     print(bot.arm.set_joint_positions(transcaction))


#     # print(bot.arm.set_joint_positions(transcaction))
#     # print(bot.arm.set_joint_positions(middle))
#     # print(bot.arm.set_joint_positions(Home))

#     for i in range(7):
#         print(bot.arm.set_joint_positions(GATES[i]))
#         print(bot.arm.set_single_joint_position(joint_name='waist', position=-1.50))
#         input()
#         print(bot.arm.set_joint_positions(transcaction))
#         input()

#     print(bot.arm.set_joint_positions(middle))
#     print(bot.arm.set_joint_positions(Home))



#     bot.arm.go_to_sleep_pose()

#     robot_shutdown()


# if __name__ == '__main__':
#     main()
