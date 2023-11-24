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
        self.refiner=LLM_Object("refiner_context.prompt",MODEL_TO_USE,0) 
        self.judger=LLM_Object("judge_context.prompt",MODEL_TO_USE, 0) 
        self.extractor=LLM_Object("extractor_context.prompt",MODEL_TO_USE,0) 
        self.new_request=True
        self.attempts_to_understand=0
        self.target=""
        self.rolling_context=[]
        self.last_suggestion=""
        # define publishing
        self.target_publisher_=self.create_publisher(String,'target',10)
        self.chat_publisher_=self.create_publisher(String, 'clarifying_question',10)
        #define subscriptions
        self.subscriber_=self.create_subscription(String, 'user_request',self.get_request,10)

    def get_request(self, request_msg: String): #callback for subscribing to user response
        #this callback gets the user response to a chat string
        #check if the response is yes-that means user is confirming your last item,
        #if so, the last ai message is the target
        request=request_msg.data

        if request.strip().lower()=="yes" and self.new_request==False:
            #user is responding to a direct suggestion
            self.target=extract_request(self.refiner.last_suggestion) 
            self.broadcast_target(self.target)

        self.new_request=False         

        #SO if not responding yes, use judge to see if the item is actionable in response
        judgement=judge(self.judger, request)
        print("judgement= "+ judgement)
        if judgement=="yes":
            #identify simple target
            self.target=extract_request(self.extractor, request)
            print(self.target)
            #publish target
            self.broadcast_target(self.target)

        #if we've been going round, restart conversation
        elif self.attempts_to_understand==5:
            #reset rolling context
            self.refiner.reset_context()
            self.attempts_to_understand=0
            #Chat the request to start over
            start_over_msg=String()
            start_over_msg.data="I'm sorry i dont inderstand.  Could we start over?"
            self.chat_publisher_.publish(start_over_msg)
        else:
            #we dont know what the target is yet so ask another question
            self.ask_clarifying_question(request)
    
    def ask_clarifying_question(self, user_msg:str):
        self.rolling_context.append(HumanMessage(content=user_msg))
        #LLM call to generate clarifying qustion
        ai_question_to_user=self.refiner.chat(self.rolling_context)
        self.last_suggestion=ai_question_to_user.content
        self.get_logger().info( "The response from the LLM is "+ai_question_to_user.content)
        self.rolling_context.append(ai_question_to_user.content)
        self.attempts_to_understand+=1
        #publish the question to user
        clarifying_question=String()
        clarifying_question.data=ai_question_to_user.content
        self.chat_publisher_.publish(clarifying_question)

    def broadcast_target(self, target:str): #callback for publishing target name
        self.get_logger().info("The target is "+ self.target)
        target_msg=String()
        target_msg.data=target
        self.target_publisher_.publish(target_msg)
        self.reset()

    def reset(self):
        self.new_request=True
        self.attempts_to_understand=0
        self.target=""
        self.rolling_context=[]
        self.refiner.reset_context()



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
    response=response.replace("a ","")
    return response

def main(args=None):
    rclpy.init(args=args)
    interpreter=TerpNode()
    rclpy.spin(interpreter)
    interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
