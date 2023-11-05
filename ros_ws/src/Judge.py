#This is the judge to determine if the response from the Refiner 
#chat is sufficiently specific to publish to task planner

import os
import langchain
from langchain import PromptTemplate
from langchain.prompts import (ChatPromptTemplate,PromptTemplate,SystemMessagePromptTemplate, HumanMessagePromptTemplate)
from langchain.chat_models import ChatOpenAI
from langchain.chains import LLMChain
from langchain.llms import OpenAI

api_key=os.environ["OPENAI_API_KEY"]

def Judge(request):
    llm = OpenAI(model_name="text-davinci-003", temperature=0)

    context="""An agent is attempting to get a specific item requested by a human user.  
        The human user is sometimes vague and the agent is trying to clarify specifically what the 
        human user wants.  You are an AI helping the agent to judge if the item 
        requested is specific enough to search for.  
        You are also an expert in grammatical structures and will be given a sentence from which 
        you should extract what exactly is being requested. It will likely be the direct object in a request 
        or question or the subject of a statement.  
        If the response has the word "anything" or "something" in it, it is not specific enough.
        If the object is specific enough to search for, respond  with "yes", otherwise respond with "no".  Do not be verbose. 
        """
    
    '''context="""An agent is attempting to get a specific item requested by a human user.  
        The human user is sometimes vague and the AI is trying to clarify specifically what the 
        human user wants.  You are an AI helping the agent to judge if the item responded is specific enough to search
        for.  If it is specific enough to search for, respond  with "yes", otherwise respond with "no".  Do not be verbose."""
    '''
    '''context="""An agent is attempting to get a specific item requested by a human user.  
        The human user is sometimes vague and the AI is trying to clarify specifically what the 
        human user wants.  You are an AI helping the agent to judge if the item responded is specigic enough to search
        for.  If it is specific enough to search for, respond  with yes, otherwise respond with no.  Do not be verbose."""
   '''
    
    system_message_prompt=SystemMessagePromptTemplate.from_template(context)
    human_template=f"Am I specifically requesting an object, if so what? : {request}"
    human_message_prompt=HumanMessagePromptTemplate.from_template(human_template)
    

    chat_prompt=ChatPromptTemplate.from_messages([system_message_prompt,human_message_prompt])
    chain=LLMChain(llm=llm, prompt=chat_prompt)
    response=chain.run(question=request)
    response=response.replace("\n","")
    response=response.replace("Answer: ","")
    response=response.replace(".","")


    return response

if __name__=="__main__":
    #request= "bottle of water"
    #request="something cold to drink"
    request="I'm thirsty"

    print(Judge(request))
