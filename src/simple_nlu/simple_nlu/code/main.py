import os
import pickle
import openai
from functools import partial
from langchain.embeddings.openai import OpenAIEmbeddings
from RAG import getRetriever, queryVectorDatabase
from dotenv import load_dotenv


# Verifique que o caminho do seu terminal está correto.
load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")
embedding = OpenAIEmbeddings(openai_api_key=os.environ.get('OPENAI_API_KEY'))

# Loads already created dbEngine. If new documents are added, run redoEmbeddings.py
with open('dbEmbeddings.pkl', 'rb') as file:
    dbEngine = pickle.load(file)

retriever = getRetriever(embedding, dbEngine)

query = partial(queryVectorDatabase, retriever=retriever)

r = query("Porque há uma inconformidade na regra 4?")
print(r)
