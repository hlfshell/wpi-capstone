import os
import faiss
import langchain

os.environ["OPENAI_API_KEY"] = "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"
from langchain import PromptTemplate
from langchain.prompts import (ChatPromptTemplate,PromptTemplate,SystemMessagePromptTemplate, HumanMessagePromptTemplate)
from langchain.agents import load_tools
from langchain.agents import initialize_agent
from langchain.agents import AgentType
from langchain.agents.load_tools import get_all_tool_names
from langchain.chains import LLMChain
from langchain import ConversationChain
from langchain.utilities import GoogleSearchAPIWrapper
from langchain.document_loaders import Docx2txtLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.vectorstores import FAISS
from langchain.llms import OpenAI
#import streamlit as st
from pathlib import Path

''' THis creates the embedding from a doc
def create_db_from_brain(path="KB.docx"):
    loader=Docx2txtLoader(path)
    text=loader.load()

    text_splitter=RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=100)
    docs=text_splitter.split_documents(text)
    embeddings=OpenAIEmbeddings()
    db=FAISS.from_documents(docs, embeddings)
    return db
    '''


def get_response_from_query(db, query, k=4):
    docs=db.similarity_search(query, k=k)
    docs_page_content=" ".join([d.page_content for d in docs])
    llm = OpenAI(model_name="text-davinci-003", temperature=0)

    template="""
            You are a home assistant that can identify an item that is being requested by
            your human user who is likely not physically capable of retrieving the item by
            themself.  You {docs}
            
            """
    system_message_prompt=SystemMessagePromptTemplate.from_template(template)
    human_template=f"Isolate what I am requesting: {query}"
    human_message_prompt=HumanMessagePromptTemplate.from_template(human_template)

    chat_prompt=ChatPromptTemplate.from_messages([system_message_prompt,human_message_prompt])
    chain=LLMChain(llm=llm, prompt=chat_prompt)
    response=chain.run(question=query, docs=docs_page_content)
    response=response.replace("\n","")
    return response, docs

# First, let's load the language model we're going to use to control the agent.
#llm = OpenAI(model_name="text-davinci-003", temperature=0)
#loop till exit

db=create_db_from_brain()
#st.title('Jackrabbit PartBot')
#query=st.text_input('How can I help you?')
query=input("How can I help you?   ")
while query:
    response, docs=get_response_from_query(db,query)
    print(response)
#   st.write(response)
#   query=st.text_input('How can I help you?')
    query=input("How can I help you?   ")
