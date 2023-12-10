from __future__ import annotations
from time import sleep
from typing import List, Tuple

import rclpy
from gazebo_msgs.srv import DeleteEntity, GetEntityState, GetModelList, SpawnEntity
from rclpy.node import Node

from litterbug.items import Item


class Gazebo(Node):
    """
    Gazebo is a helper class that manages the interaction with the
    gazebo simulator, handling the process of checking on, adding,
    and removing items, and so on.
    """

    def __init__(self, models_dir: str = "./models"):
        super().__init__("gazebo_item_service")

        self.__models_dir = models_dir

        self.__spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.__delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
        self.__get_model_list_client = self.create_client(
            GetModelList, "/get_model_list"
        )
        self.__get_entity_state_client = self.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )
        # Test
        self.__get_entity_exists_client = self.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )

    def wait_for_ready(self):
        """
        wait_for_ready spins until each client is ready to operate
        """
        self.__spawn_entity_client.wait_for_service()
        self.__delete_entity_client.wait_for_service()
        self.__get_model_list_client.wait_for_service()
        self.__get_entity_state_client.wait_for_service()
        self.__get_entity_exists_client.wait_for_service()

    def spawn_item(self, item: Item, logger):
        """
        spawn_item spawns an item into the world
        """
        if self.item_exists(item):
            logger.info(f"Item {item.name} already exists; can not create")
            raise ItemAlreadyExists(item)

        request = SpawnEntity.Request()
        request.name = item.name
        request.xml = open(f"{self.__models_dir}/{item.model}", "r").read()

        x, y, z = item.origin

        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        x, y, z, w = item.orientation

        request.initial_pose.orientation.x = x
        request.initial_pose.orientation.y = y
        request.initial_pose.orientation.z = z
        request.initial_pose.orientation.w = w

        future = self.__spawn_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: SpawnEntity.Response = future.result()

        if not response.success:
            logger.info(f"Could not add item {item.name} - {response.status_message}")
            raise CouldNotAddItem(item, response.status_message)
        logger.info(
            f"Added item {item.name} successfully {response.success} - {response.status_message}"
        )

    def item_exists(self, item: Item) -> bool:
        """
        item_exists checks if an item exists in the world
        """
        request = GetEntityState.Request()
        request.name = item.name

        future = self.__get_entity_exists_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        return response.success

    def delete_item(self, item: Item):
        """
        delete_item removes an item from the world *if it exists*.
        """
        if not self.item_exists(item):
            raise ItemDoesNotExist(item)

        request = DeleteEntity.Request()
        request.name = item.name

        future = self.__delete_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: DeleteEntity.Response = future.result()
        if not response.success:
            raise CouldNotDeleteItem(item, response.status_message)

    def get_model_list(self) -> List[Item]:
        """
        list_items returns a list of items that exist in the world
        """
        request = GetModelList.Request()
        future = self.__get_model_list_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GetModelList.Response = future.result()

        if not response.success:
            raise CouldNotListModels(response.status_message)

        items: List[Item] = []
        for model_name in response.model_names:
            position, orientation = self.get_model(model_name)

            # Create an item placeholder for the information,
            # even though we're missing label and model
            item = Item(
                name=model_name,
                label="",
                model="",
                origin=position,
                orientation=orientation,
            )
            items.append(item)

        return items

    def get_model(
        self, model_name: str
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """
        Given a model name, get_model returns the the origin and the
        orientation of the model if it exists
        """
        request = GetEntityState.Request()
        request.name = model_name

        future = self.__get_entity_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: GetEntityState.Response = future.result()

        if not response.success:
            raise CouldNotGetModel(model_name, response.status_message)

        return (
            (
                response.state.pose.position.x,
                response.state.pose.position.y,
                response.state.pose.position.z,
            ),
            (
                response.state.pose.orientation.x,
                response.state.pose.orientation.y,
                response.state.pose.orientation.z,
                response.state.pose.orientation.w,
            ),
        )

    def generate_csv(self, filepath: str):
        """
        Calls the get_model_list function and writes the results
        to a csv for further editing.
        """
        items = self.get_model_list()
        Item.to_csv(items, filepath)

    def clone(self) -> Gazebo:
        """
        Returns a clone of the current Gazebo instance
        with separate clients.
        """
        return Gazebo(self.__models_dir)


class ItemAlreadyExists(Exception):
    """
    ItemAlreadyExists is an exception that is raised when an item
    is attempted to be added to the gazebo world, but it already
    exists.
    """

    def __init__(self, item: Item):
        self.item = item

    def __str__(self):
        return f"Item {self.item.name} already exists; can not create"


class ItemDoesNotExist(Exception):
    """
    ItemDoesNotExist is an exception that is raised when an item
    is attempted to be removed from the gazebo world, but it does
    not exist.
    """

    def __init__(self, item: Item):
        self.item = item

    def __str__(self):
        return f"Item {self.item.name} does not exist"


class CouldNotAddItem(Exception):
    """
    CouldNotAddItem is an exception that is raised when an item
    is attempted to be added to the gazebo world, but it fails
    for some reason.
    """

    def __init__(self, item: Item, status_message: str):
        self.item = item
        self.status_message = status_message

    def __str__(self):
        return f"Could not add item {self.item.name} - {self.status_message}"


class CouldNotDeleteItem(Exception):
    """
    CouldNotDeleteItem is an exception that is raised when an item
    is attempted to be removed from the gazebo world, but it fails
    for some reason.
    """

    def __init__(self, item: Item, status_message: str):
        self.item = item
        self.status_message = status_message

    def __str__(self):
        return f"Could not delete item {self.item.name} - {self.status_message}"


class CouldNotListModels(Exception):
    """
    CouldNotListModels is an exception that is raised when an item
    is attempted to be listed from the gazebo world, but it fails
    for some reason.
    """

    def __init__(self, status_message: str):
        self.status_message = status_message

    def __str__(self):
        return f"Could not list models - {self.status_message}"


class CouldNotGetModel(Exception):
    """
    CouldNotGetModel is an exception that is raised when an item
    is attempted to be listed from the gazebo world, but it fails
    for some reason.
    """

    def __init__(self, model_name: str, status_message: str):
        self.model_name = model_name
        self.status_message = status_message

    def __str__(self):
        return f"Could not get model {self.model_name} - {self.status_message}"
