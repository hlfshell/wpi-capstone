import rclpy
from rclpy.node import Node
from capstone_interfaces.srv import LLM

from llm.providers import OpenAI, PaLM


class LLMService(Node):
    def __init__(self, provider):
        super().__init__("llm_service")

        # Determine the provider; this will crashif an environment variable
        # for the provider is not set
        if provider == "openai":
            self.provider = OpenAI(None, None)
        elif provider == "palm":
            self.provider = PaLM(None, None)
        else:
            raise ValueError(f"Unknown provider {provider}")

        self.srv = self.create_service(LLM, "prompt", self.prompt_callback)

    def prompt_callback(self, request: LLM.Request, response: LLM.Response):
        self.get_logger().debug(request.prompt)
        self.get_logger().debug(request.temperature)

        result = self.provider.prompt(request.prompt, request.temperature)

        self.get_logger().debug(result)

        response.result = result

        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    service = LLMService("openai")
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()
