import sys

from simple_interfaces.srv import TTS
import rclpy
from rclpy.node import Node


class TtsClient(Node):

    def __init__(self):
        super().__init__('simple_tts_client')
        self.cli = self.create_client(TTS, 'text_to_speech')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TTS.Request()

    def send_request(self, text = None): 
        if not text:
            # If the request is empty, send default text.
            self.req.text = "Hello, I am Miss Piggy, the at home robot from pe key mecanico."
        else:
            self.req.text = text
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()