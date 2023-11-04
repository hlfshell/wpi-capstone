import os
import langchain
from Judge import Judge
from Extracter import extract_request

from langchain import PromptTemplate
from langchain import schema
from langchain.prompts import (ChatPromptTemplate,PromptTemplate,SystemMessagePromptTemplate, HumanMessagePromptTemplate)
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage, AIMessage
api_key=os.environ["OPENAI_API_KEY"]

def Refine(request):
    chat = ChatOpenAI()

    sys_msg=SystemMessage(content="""You are a home assistant robot that can retrieve objects for your user. 
        You are looking for a single item to search for.  You should ask clarifying 
        questions of your user, only if needed, until you have a sufficient description of 
        a specific item to search for and retrieve.
        If the item seems vague or general or the user isnt sure, make a suggestion.
        If the user requests something or anything, you shouold make a suggestion that is consistent
        with the rest of the request.  For example, if the user requests please bring me anything cold,
        you might suggest "would you like a glass of water?" """)
   
    unknown_request=True
    context=[sys_msg]
    attempts_to_understand=0
    user_msg="User: " + request

    #If response sufficiently refined 
    #break out and pass to publish
    judgement=Judge(request).lower()
    if judgement.lower()=="yes":
        #pubish response
        target=extract_request(request)
        print("The target is ",target)
        unknown_request=False
    
    while unknown_request:
        #add user request or claification to the conversation
        context.append(HumanMessage(content=user_msg))
        response=chat(context)
        print( response.content)
        attempts_to_understand+=1

        #seek input from user 
        request=input("?")
        #If response sufficiently refined 
        #break out and pass to publish
        if request.lower()=="yes" or Judge(response.content.lower())=="yes":
            #identify simple target
            target=extract_request(response.content)
            #pubish target
            print("The target is ",target)
            break

        #if there have been too many cycles restart the request
        if attempts_to_understand==5:
            print("I'm sorry i dont inderstand.  Could we start over?")
            break
        context.append(response)



if __name__=="__main__":
    #request="Please get me a bottle of water"
    #request="Please get me a something cold to drink"
    request="I'm thirsty"

    #request="Please bring me my comb"
    #request="please get me something to fix my hair"
    #request="My hair is messy"

    #request="Please bring me the TV remote"
    #request="I'd like to turn on the TV"
    #request="Id like to watch something"

    print(Refine(request))
