from rclpy.node import Node
from simple_interfaces.srv import NLU
import rclpy

from langchain.chains import RetrievalQA
from langchain.chat_models import ChatOpenAI
from langchain.embeddings.openai import OpenAIEmbeddings
from dotenv import load_dotenv
import pickle
import openai
import os
from .code.RAG import getRetriever, queryVectorDatabase

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

class NLUServiceServer(Node):

    def __init__(self):
        super().__init__('nlu_server')
        self.srv = self.create_service(NLU, 'nlu', self.execute_callback)

        self.qa = self.setup_callback()

    def execute_callback(self, request, response):
        self.get_logger().info('Thinking...')

        r = self.qa({"query": request.query})
        answer = r["result"]

        response.answer = ""
        if answer:
            response.answer += f"{answer}"

        self.get_logger().info(f'{response}')
        return response
    
    def setup_callback(self):

        embedding = OpenAIEmbeddings(openai_api_key=os.environ.get('OPENAI_API_KEY'))

        current_directory = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(current_directory, 'code', 'dbEmbeddings.pkl')

        with open(file_path, 'rb') as file:
            dbEngine = pickle.load(file)

        retriever = getRetriever(embedding, dbEngine)

        qa = RetrievalQA.from_chain_type(
        ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0),
        chain_type="stuff",
        retriever=retriever,
        return_source_documents=True,
        )

        return qa

def main():
    rclpy.init()

    nlu_server = NLUServiceServer()

    rclpy.spin(nlu_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
