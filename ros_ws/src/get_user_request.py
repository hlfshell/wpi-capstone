import os

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
        self.context=context
        self.temp=temperature
        self.chat=ChatOpenAI(model_name=model)
        self.system_message=SystemMessage(content=self.context)

def refine(request:str):
    refiner=LLM_Object("ros_ws/src/refiner_context.txt",MODEL_TO_USE,0)
   
    unknown_request=True

    context=[refiner.system_message]
    attempts_to_understand=0

    #this is the users original request
    user_msg="User: " + request
    print (user_msg)

    #If response sufficiently refined 
    #break out and pass to publish
    judgement=judge(request)
    if judgement=="yes":
        #pubish response
        target=extract_request(request)
        print("The target is ",target)
        unknown_request=False

    #if the initial reques isnt sufficiently clear, begin chat to refine request
    while unknown_request:
        #add user request or claification to the conversation
        context.append(HumanMessage(content=user_msg))
        response=refiner.chat(context)
        print( response.content)
        attempts_to_understand+=1

        #seek input from user 
        request=input("User:")
        #If response sufficiently refined 
        #break out and pass to publish
        judgement=judge(request)
        if request.strip().lower()=="yes":
            target=extract_request(response.content)
            print("The target is ",target)
            break
        if judgement=="yes":
            #identify simple target
            target=extract_request(request)
            #pubish target
            print("The target is ",target)
            break
        #if there have been too many cycles restart the request
        if attempts_to_understand==5:
            print("I'm sorry i dont inderstand.  Could we start over?")
            break
        context.append(response)

def judge(request:str)->str:
    #this funtion takes a request and determines if a searchable object is in it
    #returns yes or no

    judger=LLM_Object("ros_ws/src/judge_context.txt",MODEL_TO_USE, 0)

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

if __name__=="__main__":
    #request="Please get me a bottle of water"
    #request="Please get me a something cold to drink"
    #request="I'm thirsty"

    #request="Please bring me my comb"
    #request="please get me something to fix my hair"
    #request="My hair is messy"

    #request="Please bring me the TV remote"
    #request="I'd like to turn on the TV"
    request="Id like to watch something"

    print(refine(request))
