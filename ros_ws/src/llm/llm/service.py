import rclpy
from llm.providers import OpenAI, PaLM
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from capstone_interfaces.srv import LLM


class LLMService(Node):
    def __init__(self):
        super().__init__("llm_service")

        self.initialize_parameters()

        self.srv = self.create_service(LLM, "prompt", self.prompt_callback)

    def initialize_parameters(self) -> None:
        self.declare_parameter(
            "provider",
            "",
            ParameterDescriptor(
                description="Which LLM provider (openai or palm) to use"
            ),
        )

        self.declare_parameter(
            "model",
            "",
            ParameterDescriptor(
                description="Which LLM model to use (provider dependent - leave blank for default)"
            ),
        )

        self.provider = (
            self.get_parameter("provider").set_parameter_value().string_value
        )
        self.model = self.get_parameter("model").set_parameter_value().string_value

        print("Hey I got the stuff", self.provider, self.model)

    def initialize_provider(self) -> None:
        # Determine the provider; this will crash if an environment variable
        # for the provider is not set
        if self.provider == "openai":
            self.provider = OpenAI(None, self.model)
        elif self.provider == "palm":
            self.provider = PaLM(None, self.model)
        else:
            raise ValueError(f"Unknown provider {self.provider}")

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
    service = LLMService()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()
