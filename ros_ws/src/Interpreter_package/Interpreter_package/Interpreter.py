import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import langchain
from langchain import PromptTemplate, schema
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
from langchain.llms import OpenAI
from langchain.prompts import (ChatPromptTemplate, HumanMessagePromptTemplate,
                               PromptTemplate, SystemMessagePromptTemplate)
from langchain.schema import AIMessage, HumanMessage, SystemMessage

OpenAI.api_key=os.environ["OPENAI_API_KEY"]
MODEL_TO_USE="gpt-4"

class LLM_Object:
    def __init__(self, context_file: str, model:str, temperature:int):
        with open(context_file, 'r') as file:
            context= file.read().replace('\n', '')
        self.rolling_context=self.context
        self.temp=temperature
        self.chat=ChatOpenAI(model_name=model)
        self.system_message=SystemMessage(content=context)

    def reset_context(self):
        self.rolling_context=[self.system_message]
        
class TerpNode(Node):
    def __init__(self):
        super().__init__('interpreter')
        self.refiner=LLM_Object("ros_ws/src/refiner_context.txt",MODEL_TO_USE,0)
        self.judger=LLM_Object("ros_ws/src/judge_context.txt",MODEL_TO_USE, 0)
        self.extractor=LLM_Object("ros_ws/src/extractor_context.txt",MODEL_TO_USE,0)
        self.unknown_request=True
        self.attempts_to_understand=0
        self.target=""

        self.target_publisher_=self.create_publisher(String,'target',10)
        self.chat_publisher_=self.create_publisher(String, 'clarifying_question',10)

        self.subscriber_=self.create_subscription(String, 'user_request',self.get_inital_request)
        self.subscriber_=self.create_subscription(String, 'cq_response',self.get_chat_response)


    def get_initial_request(self, request: String): #callback for subscribing to user initial_request
         #this is the users original/new request
         #separated initial request bc it will redirect all activites and may occur when 
         #a search has already started.  It also resets the number of attempts at understanding
         #as well as the chat history

        user_msg="User: " + request
        print (user_msg)

        self.target=""
        self.unknown_request=True
        self.attempts_to_understand=0
        #check request for clarity
        judgement=judge(self.judger, request)
        if judgement=="yes":
            #pubish the target from the initial request
            self.target=extract_request(self.extractor, request)
            print("The target is ",self.target)
            self.unknown_request=False
            self.broadcast_target(self.target)

        #if not, ask clarifying question
        else:
            self.ask_clarifying_question(user_msg)

    def get_chat_response(self, response: String): #callback for subscribing to user response
        #check if the response is yes-that means user is confirming your last item
        if response.strip().lower()=="yes":
            self.target=extract_request(response.content)
            self.broadcast_target(self.target)
            # this should be logger-print("The target is ",target)
                    
        #use judge to see if the item is clear
        judgement=self.judger(response)
        if judgement=="yes":
            #identify simple target
            self.target=extract_request(response)
            #pubish target
            self.broadcast_target(self.target)
            # this should be logger-print("The target is ",target)
        
        elif self.attempts_to_understand==5:
            #reset rolling context
            self.refiner.reset_context()
            self.attempts_to_understand=0
            #Chat the request to start over
            self.chat_publisher.publish("I'm sorry i dont inderstand.  Could we start over?")
        else:
            self.rolling_context.append(response)


    def ask_clarifying_question(self, user_msg:str):
        self.rolling_context.append(HumanMessage(content=user_msg))
        ai_question=self.refiner.chat(self.rolling_context)
        print( ai_question.content)
        self.attempts_to_understand+=1
        self.chat_publisher.publish(ai_question)

    def broadcast_target(self, target:str): #callback for publishing target name
        self.target_publisher.publish(target)


def judge(judger:LLM_Object, request:str)->str:
    #this funtion takes a request and determines if a searchable object is in it
    #returns yes or no
    #it only needs to be called once bc the context never changes

    system_message_prompt=SystemMessagePromptTemplate.from_template(judger.context)
    human_template=f"Am I specifically requesting an object, if so what? : {request}"
    human_message_prompt=HumanMessagePromptTemplate.from_template(human_template)

    chat_prompt=ChatPromptTemplate.from_messages([system_message_prompt,human_message_prompt])
    chain=LLMChain(llm=judger.chat, prompt=chat_prompt)
    response=chain.run(question=request)
    response=cleanup(response)
    return response

def extract_request(verbose_request:str)->str:
    #this function takes a long request and extracts the target object
    #returning the object and its descriptors

    extractor=LLM_Object("ros_ws/src/extractor_context.txt",MODEL_TO_USE,0)

    system_message_prompt=SystemMessagePromptTemplate.from_template(extractor.context)
    human_template=f"Isolate what I am requesting: {verbose_request}"
    human_message_prompt=HumanMessagePromptTemplate.from_template(human_template)
    

    chat_prompt=ChatPromptTemplate.from_messages([system_message_prompt,human_message_prompt])
    chain=LLMChain(llm=extractor.chat, prompt=chat_prompt)
    response=chain.run(question=verbose_request)
    response=cleanup(response)

    return response

def cleanup(response):
    response=response.replace("\n","")
    response=response.replace("Answer: ","")
    response=response.replace(".","")
    response=response.strip()
    response=response.lower()
    return response

def main(args=None):
    rclpy.init(args=args)
    interpreter=TerpNode()
    rclpy.spin(interpreter)

    interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
