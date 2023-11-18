import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class User_Node(Node):
    def __init__(self):
        super().__init__('user')
        self.request_publisher_=self.create_publisher(String,'user_request',10)
        self.chat_publisher_=self.create_publisher(String, 'cq_response',10)
        #define subscriptions
        self.subscriber_=self.create_subscription(String, 'clarifying_question',self.get_question_and_respond)

    def initiate_request(self):
        request=input('What would you like?')
        self.request_publisher_.publish(request)

    def get_question_and_respond(self, question:str):
        print(question)
        response=input('?:')
        self.chat_publisher_(response)
    
def main(args=None):
    rclpy.init(args=args)
    user=User_Node()
    user.initiate_request()
    rclpy.spin()
    user.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
