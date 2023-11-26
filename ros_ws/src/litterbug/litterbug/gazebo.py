from rclpy.node import Node
from gazebo_msgs.srv import (
    SpawnEntity,
    GetWorldProperties,
    GetModelList,
    GetModelState,
    GetEntityState,
    GetModelProperties,
    DeleteEntity,
)
from litterbug.items import Item
from typing import List


class Gazebo(Node):
    """
    Gazebo is a helper class that manages the interaction with the
    gazebo simulator, handling the process of checking on, adding,
    and removing items, and so on.
    """

    def __init__(self):
        super().__init__("gazebo_item_service")

        self.__spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.__delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
        self.__get_model_list_client = self.create_client(
            GetModelList, "/get_model_list"
        )
        self.__get_entity_state_client = self.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )

    def item_exists(self, item: Item) -> bool:
        """
        item_exists checks if an item exists in the wworld
        """
        request = GetEntityState.Request()
        request.name = item.name

        response = self.__get_entity_state_client.call(request)
        return response.success

    def add_model(self, item: Item):
        """
        add_model adds an item to the world *if it does not already
        exist*. It will raise an ItemAlreadyExists exception if so.
        If the item can not be added for another reason, a
        CouldNotAddItem exception will be raised.
        """
        if self.item_exists(item):
            raise ItemAlreadyExists(item)

        request = SpawnEntity.Request()
        request.name = item.name

        # TODO - pose

        response = self.__spawn_entity_client.call(request)
        if not response.success:
            raise CouldNotAddItem(item)

    def delete_model(self, item: Item):
        """
        delete_model removes an item from the world *if it exists*.
        It will raise an ItemDoesNotExist exception if so. If the
        item can not be removed for another reason, a
        CouldNotDeleteItem exception will be raised.
        """
        if not self.item_exists(item):
            raise ItemDoesNotExist(item)

        request = DeleteEntity.Request()
        request.name = item.name

        response = self.__delete_entity_client.call(request)
        if not response.success:
            raise CouldNotDeleteItem(item)

    def list_items(self) -> List[Item]:
        """
        list_items returns a list of items that exist in the world
        """
        request = GetModelList.Request()
        response = self.__get_model_list_client.call(request)
        # Convert the response into Items
        ## TODO
        return []

    def wait_for_ready(self):
        """
        wait_for_ready spins until each client is ready to operate
        """
        self.__spawn_entity_client.wait_for_service()
        self.__delete_entity_client.wait_for_service()
        self.__get_model_list_client.wait_for_service()
        self.__get_entity_state_client.wait_for_service()


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

    def __init__(self, item: Item):
        self.item = item

    def __str__(self):
        return f"Could not add item {self.item.name}"


class CouldNotDeleteItem(Exception):
    """
    CouldNotDeleteItem is an exception that is raised when an item
    is attempted to be removed from the gazebo world, but it fails
    for some reason.
    """

    def __init__(self, item: Item):
        self.item = item

    def __str__(self):
        return f"Could not delete item {self.item.name}"
