import rclpy
from rclpy.node import Node
import threading

from sensor_msgs.msg import JointState

joint_name=[None] * 5
joint_pos=[None] * 5

def service_thread(node):
    rclpy.spin(node)

class joint_sub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'wx250/joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global joint_name,joint_pos

        for i in range(5):

            joint_name[i]=msg.name[i]
            joint_pos[i]=msg.position[i]

        # print(f"Joint names: {joint_name}, joint positions: {joint_pos}")
        # for i in range(5):
        #     print(f"{joint_name[i]}: POS: {joint_pos[i]}  ",end="")

        # print("")
def main(args=None):
    rclpy.init(args=args)

    # Create node here
    joint_subscriber = joint_sub()

    # Start the spinning thread
    service_thread_handle = threading.Thread(target=service_thread, args=(joint_subscriber,))
    service_thread_handle.start()

    while 0==0:
        cmd=input("Enter input\n")
        print(cmd)
        if cmd=="p":
            for i in range(5):
                print(f"{joint_name[i]}: POS: {joint_pos[i]}  ",end="")
            print("")


    # Wait for the thread to finish (optional)
    service_thread_handle.join()

    # Shutdown rclpy after the thread has finished
    rclpy.shutdown()

if __name__ == '__main__':
    main()
