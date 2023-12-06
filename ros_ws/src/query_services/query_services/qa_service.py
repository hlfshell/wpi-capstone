import os
import sqlite3

import builtin_interfaces
import openai
import pandas as pd
import rclpy
from capstone_interfaces.srv import LLM, PlannerQuery
from langchain.embeddings import OpenAIEmbeddings
from query_services import database_functions, tools
from rclpy.node import Node

openai.api_key = os.getenv('OPENAI_API_KEY')

class QuestionAnswerService(Node):

    def __init__(self):
        super().__init__('question_answer_service')
        self.conn = database_functions.create_connection(self,r"state_db.db")
        self.srv = self.create_service(PlannerQuery, 'general_query', self.query)

    def query(self, question_request: PlannerQuery.Request, response: PlannerQuery.Response):
        """
        From queried question, returns response location and x,y,z
        """
        question = question_request.question   

        cur = self.conn.cursor()
        cur.execute("SELECT * FROM objects")
        rows = cur.fetchall()     

        state_list = []
        input_format = ["id","description","location","x","y","z","time seen"]

        tools.save_input_to_json(rows,input_format)
        tools.embed_documents_in_json_file("state.json")
        df = pd.read_json("state.json",orient="index")

        res = tools.search_embeddings(df,question,n=1,pprint=True,n_lines=1)

        for r in res.iterrows(): # useful when n>1 and wnat to print top n values but only save most relevant
            most_likely_location = str(r[1].location)
            most_likely_x = float(r[1].x)
            most_likely_y = float(r[1].y)
            most_likely_z = float(r[1].z)
            break

        response.location = most_likely_location
        response.x = most_likely_x
        response.y = most_likely_y
        response.z = most_likely_z

        return response


def main():
    rclpy.init()

    qa = QuestionAnswerService()
    rclpy.spin(qa)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

