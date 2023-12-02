from capstone_interfaces.msg import ObjectSpotted,StateObject
from capstone_interfaces.srv import AddObject,ObjectDescriptionQuery
from query_services import database_functions

import numpy as np
from sklearn.cluster import KMeans

import rclpy
from rclpy.node import Node


QUEUE_SIZE = 10
THRESHOLD = .5

class AddObjectNode(Node):
    def __init__(self):
        super().__init__('new_object_node')
        self.buffer = []
        self.object_list_to_add = []

        self.subscription = self.create_subscription(
            ObjectSpotted,
            'topic',
            self.process_incoming_objects,
            QUEUE_SIZE)
        self.subscription  # prevent unused variable warning

        self.add_object_client = self.create_client(AddObject, 'add_new_object')
        while not self.add_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.add_req = AddObject.Request()

        self.description_client = self.create_client(ObjectDescriptionQuery, 'object_description_query')
        while not self.description_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.description_req = ObjectDescriptionQuery.Request()

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

    def process_incoming_objects(self,msg):
        """
        Subscription callback function.
        Takes msg, adds to buffer. 
        If buffer is >10, creates new object client and calls found object for the processed list of new objects to add to the database
        """
        
        if len(self.buffer)>10:
            self.buffer.pop(0)
            self.object_list_to_add = self.process_buffer()
 
        self.buffer.append(msg)
            
        for obj_info in self.object_list_to_add:
            response = self.found_object(obj_info)
            if response != None:
                self.get_logger().info(
                    'Object:\n  ID: %d\n  Description: %s\n  Location: %s\n  (x,y,z): (%f,%f,%f)\n  Time: %s' %
                    (response.object_state.id, response.object_state.description, response.object_state.location, 
                    response.object_state.x,response.object_state.y,response.object_state.z,response.object_state.time_seen))
        
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
        
        relevant = [obj for obj in self.buffer if descriptions.count(obj.description)>2] # takes care of false positives      
        self.get_logger().debug("Relevant Objects: "+str([r.description for r in relevant])+"\n")

        for object in relevant:
            self.buffer.remove(object)      

            response = self.object_query(object.description)
            if response != None:
                matching_descriptions = response.states_of_objects
            else: 
                matching_descriptions = []
                self.get_logger().debug("Number of matches "+str(len(matching_descriptions)))
            
            for matching_object in matching_descriptions:
                if self.euclidean_distance((matching_object.x,matching_object.y,matching_object.z),(object.x,object.y,object.z))<THRESHOLD:
                    seen_before = True
                    update_object = matching_object
                    update_object.time_seen = object.time_seen
                    relevant.remove(matching_object)
                    break
            
            if seen_before:
                conn = database_functions.create_connection(self,r"state_db.db")
                database_functions.update_task(conn, update_object)
                continue
            
            points = np.zeros((relevant.count(object),3))
            points = [(object.x,object.y,object.z) for obj in relevant if obj.description == object.description]

            if points:
                k = 1
                kmeans = KMeans(n_init='auto', n_clusters = k, random_state = 2)

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

    def euclidean_distance(self,point1,point2):
        d = np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2+(point1[2]-point2[2])**2)
        return d

        
def main():
    rclpy.init()

    vision_subscriber = AddObjectNode()
    rclpy.spin(vision_subscriber)

    vision_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
