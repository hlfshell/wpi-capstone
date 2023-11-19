import os
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

import langchain
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
from langchain.llms import OpenAI
from langchain.prompts import (ChatPromptTemplate, HumanMessagePromptTemplate,
                                SystemMessagePromptTemplate)
from langchain.schema import AIMessage, HumanMessage, SystemMessage

from ament_index_python.packages import get_package_share_directory
data_dir = os.path.join(get_package_share_directory("interpreter_package"))

OpenAI.api_key=os.environ["OPENAI_API_KEY"]
MODEL_TO_USE="gpt-4"

class LLM_Object:
    def __init__(self, context_file: str, model:str, temperature:int):
        with open(os.path.join(data_dir,context_file), 'r') as file:
            context= file.read().replace('\n', '')
        self.context=context
        self.rolling_context=self.context
        self.temp=temperature
        self.chat=ChatOpenAI(model_name=model)
        self.system_message=SystemMessage(content=self.context)

    def reset_context(self):
        self.rolling_context=[self.system_message]
        
class TerpNode(Node):
    def __init__(self):
        super().__init__('interpreter')
        self.get_logger().info("Terp Started")
        #define elements
        self.refiner=LLM_Object("refiner_context.txt",MODEL_TO_USE,0) 
        self.judger=LLM_Object("judge_context.txt",MODEL_TO_USE, 0) 
        self.extractor=LLM_Object("extractor_context.txt",MODEL_TO_USE,0) 
        self.unknown_request=True
        self.attempts_to_understand=0
        self.target=""
        self.rolling_context=[]
        # define publishing
        self.target_publisher_=self.create_publisher(String,'target',10)
        self.chat_publisher_=self.create_publisher(String, 'clarifying_question',10)
        #define subscriptions
        self.subscriber_=self.create_subscription(String, 'user_request',self.get_initial_request,10)
        self.subscriber_=self.create_subscription(String, 'cq_response',self.get_chat_response, 10)

    def get_initial_request(self, request: String): #callback for subscribing to user initial_request
         #this is the users original/new request
         #separated initial request bc it will redirect all activites and may occur when 
         #a search has already started.  It also resets the number of attempts to understand
         #as well as the chat history

        user_msg="User: " + request.data
        #self.get_logger().info("I understand the user " + user_msg)

        #initalize vars for new request
        self.target=""
        self.unknown_request=True
        self.attempts_to_understand=0

        #check request user request to see if it is clear already
        judgement=judge(self.judger, request.data)
        if judgement=="yes":
            #extract and pubish the target from the initial request
            self.target=extract_request(self.extractor, request.data)
            self.get_logger().info("The target is " + self.target)
            self.unknown_request=False
            self.broadcast_target(self.target)

        #if not, ask clarifying question
        else:
            self.ask_clarifying_question(user_msg)

    def get_chat_response(self, response_msg: String): #callback for subscribing to user response
        #this callback gets the user response to a chat string
        #check if the response is yes-that means user is confirming your last item,
        #if so, the last ai message is the target
        response=response_msg.data
        if response.strip().lower()=="yes":
            self.target=extract_request(self.refiner.AIMessage.content)
            self.broadcast_target(self.target)
            self.get_logger().info("The target is " + self.target)
                    
        #SO if not responding yes, use judge to see if the item is actionable in response
        judgement=judge(self.judger, response)
        if judgement=="yes":
            #identify simple target
            self.target=extract_request(self.extractor, response)
            #pubish target
            self.broadcast_target(self.target)
            self.get_logger().info("The target is "+ self.target)
        
        #if we've been going round, restart conversation
        elif self.attempts_to_understand==5:
            #reset rolling context
            self.refiner.reset_context()
            self.attempts_to_understand=0
            #Chat the request to start over
            self.chat_publisher_.publish("I'm sorry i dont inderstand.  Could we start over?")
        else:
            #we dont know what the target is yet so ask another question
            self.ask_clarifying_question(response)

    def ask_clarifying_question(self, user_msg:str):
        self.rolling_context.append(HumanMessage(content=user_msg))
        #LLM call to generate clarifying qustion
        ai_question_to_user=self.refiner.chat(self.rolling_context)
        self.get_logger().info( "The response from the LLM is "+ai_question_to_user.content)
        self.rolling_context.append(ai_question_to_user.content)
        self.attempts_to_understand+=1
        #publish the question to user
        clarifying_question=String()
        clarifying_question.data=ai_question_to_user.content
        self.chat_publisher_.publish(clarifying_question)

    def broadcast_target(self, target:str): #callback for publishing target name
        target_msg=String()
        target_msg.data=target
        self.target_publisher_.publish(target_msg)

    def shutdown_and_restart_node(self):
        #here we shutdown and restart the interpreter node after identifying target
        pass

def judge(judger:LLM_Object, request:String)->str:
    #this funtion takes a request and determines if a searchable object is in it
    #returns yes or no

    system_message_prompt=SystemMessagePromptTemplate.from_template(judger.context)
    human_template=f"Am I specifically requesting an object, if so what? : {request}"
    human_message_prompt=HumanMessagePromptTemplate.from_template(human_template)

    chat_prompt=ChatPromptTemplate.from_messages([system_message_prompt,human_message_prompt])
    chain=LLMChain(llm=judger.chat, prompt=chat_prompt)
    response=chain.run(question=request)
    response=cleanup(response)
    return response

def extract_request(extractor: LLM_Object, verbose_request:str)->str:
    #this function takes a long request and extracts the target object
    #returning the object and its descriptors

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
