

import os
import langchain
from langchain import PromptTemplate
from langchain.prompts import (ChatPromptTemplate,PromptTemplate,SystemMessagePromptTemplate, HumanMessagePromptTemplate)
from langchain.chat_models import ChatOpenAI
from langchain.chains import LLMChain
from langchain.llms import OpenAI

api_key=os.environ["OPENAI_API_KEY"]

def extract_request(verbose_request):
    llm = OpenAI(model_name="text-davinci-003", temperature=0)

    context=""" You are an expert in grammatical structures and will be given a sentence from which 
        you should extract what exactly is being requested. It will likely be the direct object in a question.  
        Reply with just the object and its modifiers.  For example replay with "can of soda" rahther than just "soda"
        Do not be verbose."""
    
    system_message_prompt=SystemMessagePromptTemplate.from_template(context)
    human_template=f"Isolate what I am requesting: {verbose_request}"
    human_message_prompt=HumanMessagePromptTemplate.from_template(human_template)
    

    chat_prompt=ChatPromptTemplate.from_messages([system_message_prompt,human_message_prompt])
    chain=LLMChain(llm=llm, prompt=chat_prompt)
    response=chain.run(question=verbose_request)
    response=response.replace("\n","")

    return response