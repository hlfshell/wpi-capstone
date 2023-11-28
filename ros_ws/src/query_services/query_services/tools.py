import json
import os
from langchain.embeddings import OpenAIEmbeddings
from langchain.text_splitter import RecursiveCharacterTextSplitter
from openai.embeddings_utils import cosine_similarity
from concurrent.futures import ThreadPoolExecutor
import requests
from bs4 import BeautifulSoup
import re
import io
import openai

### TOOLS FOR EMBEDDINGS FROM USER INPUT ###

def save_input_to_json(state_db,format_template)->None:
    count = 1
    # state_object = ""
    if not os.path.exists("state.json"):
        # count = 1
        with open("state.json","w") as f:
            json.dump({},f,ensure_ascii=False,indent=4,separators=(',',': '))

    else:
        with open("state.json","r") as f:
            data = json.load(f)
    data = {}
    state_object = {}
    
    for state_input in state_db:
        for entry in state_input:
            format_type = format_template[state_input.index(entry)]
            if format_type == "id":
                continue
            state_object[format_type] = entry
        data[count] = state_object
        with open("state.json","w") as f:
            json.dump(data,f,ensure_ascii=False,indent=4,separators=(',',': '))
        count += 1
        state_object = {}

    for c in range(count):
        if c == count-1:
            break
        
def get_embedding(text:str) -> list:
    embeddings_model = OpenAIEmbeddings()
    res = embeddings_model.embed_documents([text])[0]
    return res

def embed_chunk(i:int,text):
    embeddings = get_embedding(text)
    return i,embeddings

def embed_documents_in_json_file(file_path:str)-> None:
    with open(file_path,"r",encoding="utf-8") as f:
        data = json.load(f)

    # Create a ThreadPoolExecutor
    with ThreadPoolExecutor() as executor:
        # List to hold future objects
        futures = []

        # Iterate through the data, submitting each chunk to the executor
        for i in range(len(data)):
            if "embeddings" not in data[str(i+1)].keys():
                text = data[str(i+1)]["description"]
                future = executor.submit(embed_chunk,i,text)
                futures.append(future)

        # Retrieve the results from the futures, keeping them in order
        for future in futures:
            i, embeddings = future.result()
            data[str(i+1)]["embeddings"] = embeddings

    with open(file_path,"w",encoding="utf-8") as f:
        json.dump(data,f,ensure_ascii=False,indent=4,separators=(',',': '))
    QuestionAnswerService.get_logger().debug("Embeddings Saved")

def search_embeddings(df,query,n=3,pprint=True,n_lines=1):
    embedding = get_embedding(query)
    df['similarities'] = df.embeddings.apply(lambda x: cosine_similarity(x,embedding))

    res = df.sort_values('similarities',ascending=False).head(n)
    if pprint:
        for r in res.iterrows():
            # print n_lines of the context
            QuestionAnswerService.get_logger().info(r[1].description.split('\n')[:n_lines],'Similarity:',r[1].similarities)
    return res

def split_text(text:str,chunk_character_count:int = 1000,chunk_overlap: int=100):
    # Initialize the text splitter with custom parameters
    custom_text_splitter = RecursiveCharacterTextSplitter(
        # Set custom chunk size
        chunk_size = chunk_character_count,
        chunk_overlap=chunk_overlap,
        # Use length of text as size measure
        length_function=len,
    )

    # Create the chunks
    texts = custom_text_splitter.create_documents([text])
    return texts