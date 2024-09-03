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
        middle = [-2.02, -0.65, 0.33, -0.19, -0.06]
        ingame = [-1.70, -0.65, -0.54, 1, -0.75]

        left_look = [-0.003,-1.33,0.68,-0.9, 0.02] 
        center_look = [-0.003,-1.33,0.68,-0.9, -1.53]
        right_look = [-0.003,-1.33,0.68,-0.9, -3.12]

        self.POSITIONS = {
            "sleep": None,
            "home": home,
            "midle": middle,
            "ingame":ingame,
            "dick":[0,0,-0.8,0,0],#in your ass :)

            "left_look":[-0.003,-1.33,0.68,-0.9, 0.02] ,
            "center_look": [-0.003,-1.33,0.68,-0.9, -1.53],
            "right_look": [-0.003,-1.33,0.68,-0.9, -3.12],

            # [-1.70, 0.88, -1.8, 0.78, 0.5],
            # [-1.70, 0.15, -0.5,    0.4, 0.05],
            # [-1.70, -0.19, 0.06,   0.05, 0.05],
            # [-1.70, -0.3, -0.11,   0.8, 0.05],
            # [-1.70,-0.28,-0.4,     1.545, 0.8],
            # [-1.70,-0.43,-0.355,   1.76, 0.05],
            # [-1.61, -0.67, -0.23,  1.85, 0.05],



            "gate0": [-1.70, 0.425, -0.79, 0.17, 0.004601942375302315],
            "gate1": [-1.70, 0.15, -0.5,   0.4, -0.015339808538556099],
            "gate2": [-1.70, -0.19, 0.06,  0.05, -0.019941750913858414],
            "gate3": [-1.70, -0.3, -0.11,  0.8, 0.02454369328916073],
            "gate4": [-1.70,-0.28,-0.4,    1.545, 0.003067961661145091],
            "gate5": [-1.70,-0.43,-0.355,  1.76, 0.006135923322290182],
            "gate6": [-1.61, -0.67, -0.23, 1.85, 0.003067961661145091],

            "push0": -1.47,
            "push1": -1.46,
            "push2": -1.46,
            "push3": -1.455,
            "push4": -1.45,
            "push5": -1.45,
            "push6": -1.45,

            "loadmidle":[1.3, -1.64, 0.96, 0.62, -1.51],

            "loader0": [0.89, -1.51, 1.03, 0.48, -1.51],
            "loadpush0": [0.89, -1.71, 1.25, 0.44, -1.5],
            "loader1": [1.48, -1.66, 0.94, 0.69, -1.51],
            "loadpush1": [1.49, -1.81, 1.16,0.69, -1.51]

        }
        self.head = [left_look, center_look, right_look]

        self.last_moved_to = self.POSITIONS["sleep"]

        self.servos = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]

        self.in_game = False

        self.is_loaded = False
        self.loaded_counter = 0

        self.gates_to_play = []
        self.avilable = True

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
        if self.last_moved_to == self.POSITIONS["home"]:
            self.move_to_pos("home")
            self.move_to_pos("right_look")
            sleep(0.4)
            self.move_to_pos("left_look")
            sleep(0.4)
            self.move_to_pos("center_look")
            sleep(0.1)
            self.last_moved_to = self.POSITIONS["home"]

    def start_game(self):
        if self.last_moved_to == self.POSITIONS["sleep"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["home"])
            self.bot.arm.set_joint_positions(self.POSITIONS["midle"])
            self.bot.arm.set_joint_positions(self.POSITIONS["ingame"])
        elif self.last_moved_to == self.POSITIONS["home"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["midle"])
            self.bot.arm.set_joint_positions(self.POSITIONS["ingame"])
        elif self.last_moved_to == self.POSITIONS["midle"]:
            self.bot.arm.set_joint_positions(self.POSITIONS["ingame"])
        self.last_moved_to = self.POSITIONS["ingame"]
        self.in_game = True
        

    def play_gate(self, id:int, magnets = None):
        print("thread started")
        self.gates_to_play.append(id)

        if magnets is not None:
            self.reload(magnets)

        while(not self.avilable):
            sleep(0.05)

        print("Arm available")

        self.avilable = False

        id = self.gates_to_play.pop(0)

        print(f"Dropping at gate {id}")

        self.bot.arm.set_joint_positions(self.POSITIONS[f"gate{id}"])
        sleep(0.2)
        self.move(0, self.POSITIONS[f"push{id}"])
        self.last_moved_to = self.POSITIONS[f"gate{id}"]
        if magnets is not None:
            magnets(False)
        self.move(0, -1.7)
        print("closed magnet")
        self.move_to_pos("ingame")
        print("Move done")
        self.avilable = True
        self.is_loaded = False

    def end_game(self):
        if self.in_game:
            self.move_to_pos("midle")
            self.move_to_pos("home")
            self.hibernation_mode()
            self.in_game = False
            sleep(0.3)

    def arm_is_playing(self):
        return self.in_game
    
    def reload(self, magnets):
        if(self.is_loaded == False):
            magnets(True)
            self.move_to_pos(f"loadmidle")
            self.move_to_pos(f"loader{self.loaded_counter}")
            self.move_to_pos(f"loadpush{self.loaded_counter}")
            self.move_to_pos(f"loader{self.loaded_counter}")
            self.move_to_pos("ingame")
            self.loaded_counter = (self.loaded_counter + 1)%2
            if self.loaded_counter == 0:
                print("move up the reloader to reveal the next pieces")
            self.is_loaded = True
        else:
            print("I AM ALREADY LOADED")







def main():
    arm = ROBOTIC_ARM()
    arm.move_to_pos("home")
    # arm.move_to_pos("midle")
    # arm.move_to_pos("ingame")
    arm.start_game()
    # arm.move_to_pos("loadmidle")
    # arm.move_to_pos("loader1")
    # arm.move_to_pos("loadpush1")
    # while(True):
        # id = 0

    arm.move_to_pos("ingame")
    arm.play_gate(1)

        # a = int(input("gate:"))

        # if 0<=a<=6:
        #     id = a
        #     arm.move_to_pos(f"gate{id}")
        #     servo = 0

        #     # servo = int(input("motttor: "))
        #     # radians = float(input("rsdians: "))
        #     # arm.move(servo, radians)
        #     input()
        #     # arm.move_to_pos("ingame")
        #     while(0<= servo <= 4):
        #         servo = int(input("motttor: "))
        #         radians = float(input("rsdians: "))
        #         arm.move(servo, radians)
        # # else:
        # #     break
  

if __name__ == '__main__':
    main()
