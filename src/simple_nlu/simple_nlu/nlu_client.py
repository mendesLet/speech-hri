import rclpy
import argparse
from rclpy.node import Node

from simple_interfaces.srv import NLU

class NLUClient(Node):

    def __init__(self):
        super().__init__('nlu_client')
        self.cli = self.create_client(NLU, 'nlu')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logget().info('service not available, waiting again...')
        self.req = NLU.Request()

    def send_request(self, text = None):

        self.req.query = text

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    
def arg_parser():
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '--query',
    type=str,
    help='The query to be processed by the NLU.')
  
  return parser.parse_args()
  
def main():
    args = arg_parser()
    rclpy.init()
    nlu = NLUClient()
    nlu.send_request(f'{args.query}')

if __name__ == '__main__':
    main()
