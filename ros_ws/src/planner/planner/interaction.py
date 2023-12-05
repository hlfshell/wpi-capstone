import rclpy.node

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

    def pickup_object(self, object_id: Union[str, int]):
        """
        pickup_object will call the pickup service with the given object name
        """
        if isinstance(object_id, int):
            object_id = str(object_id)

        request = PickUpObjectMsg.Request()
        request.object_name = object_id
        future = self.__pickup_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: PickUpObjectMsg.Response = future.result()
        return response.success

    def give_object(self, object_id: Union[str, int]):
        """
        give_object will call the give service with the given object name
        """
        if isinstance(object_id, int):
            object_id = str(object_id)

        request = GiveObjectMsg.Request()
        request.object_name = object_id
        future = self.__give_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GiveObjectMsg.Response = future.result()
        return response.success
