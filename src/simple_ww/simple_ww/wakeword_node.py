import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class WakeWordNode(Node):
    def __init__(self, target):
        super().__init__('wakeword_node')

        # Create a subscriber for the string topic
        self.subscriber = self.create_subscription(String, '/asr_output', self.callback, 10)

        # Create a variable to store the word to check for
        self.word_to_check = target

        # Create a variable to store the flag indicating whether the word has been found
        self.word_found = False
        self.msg = ""

    def callback(self, msg):
        self.msg = msg.data
        # Check if the word to check is in the message
        if (self.word_to_check).lower() in (msg.data).lower():
            self.word_found = True


def run_ww(target):

    ww = WakeWordNode(target)

    while not ww.word_found:
        rclpy.spin_once(ww)

    return ww.msg