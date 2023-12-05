import rclpy
from rclpy.node import Node

from capstone_interfaces.srv import PickUpObject as PickUpObjectMsg
from capstone_interfaces.srv import GiveObject as GiveObjectMsg

from typing import Union


class InteractionModule(Node):
    """
    InteractionModule handles calls for the robot to interact with
    objects in the world.
    """

    def __init__(self):
        super().__init__("interaction_module")

        self.__pickup_client = self.create_client(PickUpObjectMsg, "/pickup_object")
        self.__give_client = self.create_client(GiveObjectMsg, "/place_object")

        self.__pickup_client.wait_for_service()
        self.__give_client.wait_for_service()

    def pickup_object(self, object_id: str):
        """
        pickup_object will call the pickup service with the given object name
        """
        request = PickUpObjectMsg.Request()
        request.object = object_id
        future = self.__pickup_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: PickUpObjectMsg.Response = future.result()
        return response.success

    def give_object(self, object_id: str):
        """
        give_object will call the give service with the given object name
        """
        request = GiveObjectMsg.Request()
        request.object = object_id
        future = self.__give_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GiveObjectMsg.Response = future.result()
        return response.success
