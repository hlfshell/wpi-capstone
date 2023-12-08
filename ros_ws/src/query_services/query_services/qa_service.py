from capstone_interfaces.srv import LLM, PlannerQuery
from query_services import tools, database_functions
import os
import openai
from langchain.embeddings import OpenAIEmbeddings
import pandas as pd

import sqlite3

import rclpy
import builtin_interfaces
from rclpy.node import Node

from capstone_interfaces.msg import StateObject

from typing import List

openai.api_key = os.getenv("OPENAI_API_KEY")


class QuestionAnswerService(Node):
    def __init__(self):
        super().__init__("question_answer_service")
        self.conn = database_functions.create_connection(self, r"state_db.db")
        self.srv = self.create_service(PlannerQuery, "general_query", self.query)
        print("inited")

    def query(
        self, question_request: PlannerQuery.Request, response: PlannerQuery.Response
    ):
        """
        From queried question, returns response location and x,y,z
        """
        question = question_request.question

        cur = self.conn.cursor()
        cur.execute("SELECT * FROM objects")
        rows = cur.fetchall()

        input_format = ["id", "description", "location", "x", "y", "z", "time seen"]

        tools.save_input_to_json(rows, input_format)
        tools.embed_documents_in_json_file("state.json")
        df = pd.read_json("state.json", orient="index")

        res = tools.search_embeddings(df, question, n=25, pprint=True, n_lines=1)

        quantile = res["similarities"].quantile(0.75)
        res = res[res["similarities"] > quantile]
        print("prior", res)

        state_objects: List[StateObject] = []
        for index, row in res.iterrows():
            # self.get_logger().info(f">>> {index}, {row}")
            state_objects.append(
                StateObject(
                    id=row["id"],
                    description=row["description"],
                    location=row["location"],
                    x=row["x"],
                    y=row["y"],
                    z=row["z"],
                )
            )
        response.objects = state_objects

        return response


def main():
    rclpy.init()

    qa = QuestionAnswerService()
    rclpy.spin(qa)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
