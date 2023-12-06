from __future__ import annotations

from threading import Lock
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from capstone_interfaces.msg import ObjectSpotted, StateObject
from capstone_interfaces.srv import AddObject, ObjectDescriptionQuery
from query_services import database_functions
from query_services.database_functions import create_connection
from rclpy.node import Node
from sklearn.cluster import KMeans

QUEUE_SIZE = 10
THRESHOLD = 0.5
MIN_TIMES_SPOTTED = 15


class AddObjectNode(Node):
    def __init__(self):
        super().__init__("new_object_node")
        self.buffer = []
        self.object_list_to_add = []

        self.subscription = self.create_subscription(
            ObjectSpotted, "/object_spotted", self.handle_spotted_object, QUEUE_SIZE
        )
        self.subscription  # prevent unused variable warning

        self.add_object_client = self.create_client(AddObject, "add_new_object")
        while not self.add_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.add_req = AddObject.Request()

        self.description_client = self.create_client(
            ObjectDescriptionQuery, "object_description_query"
        )
        while not self.description_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.description_req = ObjectDescriptionQuery.Request()

        self.get_logger().info("Subscriber Initialized")

        self.__item_memory_lock: Lock = Lock()
        self.__item_memory: Dict[str, List[ObjectTracker]] = {}

        self.__init_item_memory()

    def __init_item_memory(self):
        # TODO - make this configurable to some other location
        db = create_connection(self, r"state_db.db")

        # Read all the objects into memory for faster processing
        sql = "SELECT id, description, location, x, y, z, timestamp FROM objects"
        cur = db.cursor()
        cur.execute(sql)
        rows = cur.fetchall()
        for row in rows:
            item = ObjectTracker(
                row[1], (row[3], row[4], row[5]), row[6], saved=True, id=row[0]
            )
            if row[1] not in self.__item_memory:
                self.__item_memory[row[1]] = []
            self.__item_memory[row[1]].append(item)

        # Close the connection
        db.close()

    def found_object(self, info: ObjectSpotted):
        """
        Add objects from given object information:
        : param description
        :return: all
        """
        self.add_req.object_info.description = info.description
        self.add_req.object_info.x = info.x
        self.add_req.object_info.y = info.y
        self.add_req.object_info.z = info.z
        self.add_req.object_info.time_seen = info.time_seen

        self.future = self.add_object_client.call_async(self.add_req)
        return self.future.result()

    def object_query(self, description: str) -> list[StateObject]:
        """
        Query objects by description
        Takes in description of an object to send service, returns list of StateObjects from service
        """
        self.description_req.object_description = description
        self.future = self.description_client.call_async(self.description_req)
        return self.future.result()

    def handle_spotted_object(self, msg: ObjectSpotted):
        """
        Handle incoming spotted objects and decide whether or not we need to
        add them as a new object or not.
        """
        item: ObjectTracker = None
        add_item = False
        with self.__item_memory_lock:
            if msg.description not in self.__item_memory:
                self.__item_memory[msg.description] = []

            new_object = True
            for compared_item in self.__item_memory[msg.description]:
                distance = self.euclidean_distance(
                    compared_item.location, (msg.x, msg.y, msg.z)
                )
                if distance < THRESHOLD:
                    new_object = False
                    item = compared_item
                    item.update_locale(
                        (msg.x, msg.y, msg.z), time_last_seen=msg.time_seen
                    )
                    if item.times_seen >= MIN_TIMES_SPOTTED and not item.saved:
                        add_item = True
                        item.saved = True
                    break
            if new_object:
                item = ObjectTracker(
                    msg.description, (msg.x, msg.y, msg.z), msg.time_seen
                )
                self.__item_memory[msg.description].append(item)

        # By now we know whether or not to add the item to our permanent memory
        # as a new item...
        if not add_item:
            return

        # Now let's add the item.
        final_object_spotted = ObjectSpotted()
        final_object_spotted.description = item.description
        final_object_spotted.x = item.location[0]
        final_object_spotted.y = item.location[1]
        final_object_spotted.z = item.location[2]
        final_object_spotted.time_seen = item.time_last_seen

        self.found_object(final_object_spotted)

    def process_incoming_objects(self, msg):
        """
        Subscription callback function.
        Takes msg, adds to buffer.
        If buffer is >10, creates new object client and calls found object for the processed list of new objects to add to the database
        """
        if len(self.buffer) > 10:
            self.buffer.pop(0)
            self.object_list_to_add = self.process_buffer()

        self.buffer.append(msg)

        for obj_info in self.object_list_to_add:
            response = self.found_object(obj_info)
            if response != None:
                self.get_logger().info(
                    "Object:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Time: %s"
                    % (
                        response.object_state.id,
                        response.object_state.description,
                        response.object_state.location,
                        response.object_state.x,
                        response.object_state.y,
                        response.object_state.z,
                        response.object_state.time_seen,
                    )
                )

        self.object_list_to_add = []
        if not self.object_list_to_add:
            self.get_logger().debug("No items to add")

    def process_buffer(self):
        """
        Helper function for process_incoming_objects()
        Determines relevant descriptions as those that appear more than 2x in the buffer
        Clusters points of each object type to determine the location (x,y,z) that should be added to the db

        """
        seen_before = False
        self.object_list_to_add = []
        descriptions = []
        new_obj = ObjectSpotted()

        descriptions = [obj.description for obj in self.buffer]

        relevant = [
            obj for obj in self.buffer if descriptions.count(obj.description) > 2
        ]  # takes care of false positives
        self.get_logger().debug(
            "Relevant Objects: " + str([r.description for r in relevant]) + "\n"
        )

        for object in relevant:
            self.buffer.remove(object)

            response = self.object_query(object.description)
            if response != None:
                matching_descriptions = response.states_of_objects
            else:
                matching_descriptions = []
                self.get_logger().debug(
                    "Number of matches " + str(len(matching_descriptions))
                )

            for matching_object in matching_descriptions:
                if (
                    self.euclidean_distance(
                        (matching_object.x, matching_object.y, matching_object.z),
                        (object.x, object.y, object.z),
                    )
                    < THRESHOLD
                ):
                    seen_before = True
                    update_object = matching_object
                    update_object.time_seen = object.time_seen
                    relevant.remove(matching_object)
                    break

            if seen_before:
                conn = database_functions.create_connection(self, r"state_db.db")
                database_functions.update_task(conn, update_object)
                continue

            points = np.zeros((relevant.count(object), 3))
            points = [
                (object.x, object.y, object.z)
                for obj in relevant
                if obj.description == object.description
            ]

            if points:
                k = 1
                kmeans = KMeans(n_init="auto", n_clusters=k, random_state=2)

                kmeans.fit_predict(points)
                kmeans.cluster_centers_

                new_obj.description = object.description
                new_obj.time_seen = object.time_seen

                for center in kmeans.cluster_centers_:
                    new_obj.x = center[0]
                    new_obj.y = center[1]
                    new_obj.z = center[2]
                    if new_obj not in self.object_list_to_add:
                        self.object_list_to_add.append(new_obj)

        self.get_logger().debug(str(self.object_list_to_add))
        self.object_list_to_add = self.object_list_to_add

        return self.object_list_to_add

    def euclidean_distance(self, point1, point2):
        d = np.sqrt(
            (point1[0] - point2[0]) ** 2
            + (point1[1] - point2[1]) ** 2
            + (point1[2] - point2[2]) ** 2
        )
        return d


class ObjectTracker:
    """
    ObjectTracker is a little helper class to help us with
    tracking state of objects without doing queries; useful
    for just tracking vision tracking
    """

    def __init__(
        self,
        description: str,
        location: Tuple[float, float, float],
        time_last_seen: float,
        times_seen: int = 1,
        saved: bool = False,
        id: int = None,
    ):
        self.id: int = id
        self.description: str = description
        self.location: Tuple[float, float, float] = location
        self.time_last_seen: float = time_last_seen
        self.times_seen: int = times_seen
        self.saved = saved

    def update_locale(
        self,
        location: Tuple[float, float, float],
        time_last_seen: Optional[float] = None,
    ):
        self.times_seen += 1

        if time_last_seen is not None:
            self.time_last_seen = time_last_seen

        # The following is a weighted average of the location based
        # on how many times we've seen it.
        deltax = (location[0] - self.location[0]) / self.times_seen
        deltay = (location[1] - self.location[1]) / self.times_seen
        deltaz = (location[2] - self.location[2]) / self.times_seen

        self.location = (
            self.location[0] + deltax,
            self.location[1] + deltay,
            self.location[2] + deltaz,
        )


def main():
    rclpy.init()

    vision_subscriber = AddObjectNode()
    rclpy.spin(vision_subscriber)

    vision_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
