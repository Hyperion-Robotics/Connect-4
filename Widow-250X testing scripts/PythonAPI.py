#!/usr/bin/env python3

import sys
import numpy as np
from time import sleep
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

from interbotix_xs_msgs.srv import TorqueEnable
from rclpy.node import Node

#To launch arm enter the following after sourcing the interbotix ros2 package
#ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250

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
        
        self.states = ["sleeping", "home", "action"]# sleeping: is on sleep mode, home: is on home coordinates, action: is on the transcaction coordinates 

        Home=[-0.003,-1.33,0.68,-0.9,-1.6]
        middle = [-2.02, -1.49, 0.33, -0.19, -0.06]
        transcaction = [-1.55, -1, -0.54, 1, -0.75]

        gate0 = [-1.55, 0.93, -1.8, 0.8, 0]#ok
        gate1 = [-1.55, 0.15, -0.4, 0.3, 0]#ok
        gate2 = [-1.55, 0, -0.54, 1, 0]#ok
        gate3 = [-1.55, -0.375, 0, 0.7, 0]#ok
        gate4 = [-1.55, -0.65, 0.3, 0.6, 0]#ok
        gate5 = [-1.55, -1, 0.61, 0.4, 0]#ok
        gate6 = [-1.55, -1.2, 0.45, 1, 0]#ok

        left_look = [-0.003,-1.33,0.68,-0.9, 0.02] 
        center_look = [-0.003,-1.33,0.68,-0.9, -1.53]
        right_look = [-0.003,-1.33,0.68,-0.9, -3.12]

        self.key_states = {
            "HOME": Home,
            "MIDPOINT": middle,
            "TRANSITION":transcaction
        }
        self.GATES = [gate0, gate1, gate2, gate3, gate4, gate5, gate6]
        self.head = [left_look, center_look, right_look]

        self.state = self.states[0]

        self.servos = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]

    
    def hibernation_mode(self): #the potition where the torque is closed so that the servos can rest alitle
        if self.state == self.states[0]:
            pass##torn off torque
        elif self.state == self.states[1]:
            self.bot.arm.go_to_sleep_pose()
            pass##torn off torque
        elif self.state == self.states:
            self.bot.arm.set_joint_positions(self.key_states["MIDPOINT"])
            self.bot.arm.set_joint_positions(self.key_states["HOME"])
            self.bot.arm.go_to_sleep_pose()
            pass##torn off torque

    def start_game(self):
        if self.state == self.states[0]:
            self.bot.arm.set_joint_positions(self.key_states["MIDPOINT"])
            self.bot.arm.set_joint_positions(self.key_states["TRANSITION"])
        elif self.state == self.states[1]:
            self.bot.arm.set_joint_positions(self.key_states["TRANSITION"])
        

    def play_gate(self,id:int):
        self.bot.arm.set_joint_positions(self.GATES[id])
        self.bot.arm.set_single_joint_position(joint_name=self.servos[0] , position=-1.50)
        self.bot.arm.set_joint_positions(self.key_states["TRANSITION"])






def main():

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


    # print(bot.arm.set_joint_positions(transcaction))
    # print(bot.arm.set_joint_positions(middle))
    # print(bot.arm.set_joint_positions(Home))

    for i in range(7):
        print(bot.arm.set_joint_positions(GATES[i]))
        print(bot.arm.set_single_joint_position(joint_name='waist', position=-1.50))
        input()
        print(bot.arm.set_joint_positions(transcaction))
        input()

    print(bot.arm.set_joint_positions(middle))
    print(bot.arm.set_joint_positions(Home))



    bot.arm.go_to_sleep_pose()

    robot_shutdown()


if __name__ == '__main__':
    main()
