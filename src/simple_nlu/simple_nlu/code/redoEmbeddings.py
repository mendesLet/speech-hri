import os
import pickle
import openai
from dotenv import load_dotenv
from langchain.embeddings.openai import OpenAIEmbeddings
from RAG import dbEmbeddings

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")
embedding = OpenAIEmbeddings(openai_api_key=os.environ.get('OPENAI_API_KEY'))

# Redo embeddings to account for new text in database.
dbEngine = dbEmbeddings(pathToDocs = '../docs')

with open('dbEmbeddings.pkl', 'wb') as file:
    pickle.dump(dbEngine, file)