import os
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class User_Node(Node):
    def __init__(self):
        super().__init__("user")
        self.get_logger().info("User Started")
        self.request_publisher_=self.create_publisher(String,'user_request',10)
        self.chat_publisher_=self.create_publisher(String, 'cq_response',10)
        #define subscriptions
        self.subscriber_=self.create_subscription(String, 'clarifying_question',self.get_question_and_respond, 10)
        self.subscriber_=self.create_subscription(String, 'target',self.reset_for_next_request, 10)

    def initiate_request(self):
        request=input('What can I get for you?')
        request_msg=String()
        request_msg.data=request
        self.request_publisher_.publish(request_msg)

    def get_question_and_respond(self, question:String):
        print(question.data)
        response=input('?:')
        #convert to String
        response_msg=String()
        response_msg.data=response
        self.chat_publisher_.publish(response_msg)
    
    def reset_for_next_request(self):
        #here we shut down and respawn the user node
        pass

def main(args=None):
    rclpy.init(args=args)
    user=User_Node()
    user.initiate_request()
    rclpy.spin(user)
    user.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
