"""
This file relies mostly on langchain to implement a document retriever.
"""
import os
from langchain.chains import RetrievalQA
from langchain.chat_models import ChatOpenAI
from langchain.document_loaders import TextLoader
from langchain.document_loaders import PyPDFLoader
from langchain.embeddings.openai import OpenAIEmbeddings


def dbEmbeddings(
    pathToDocs: str = "", useChroma: bool = False, chromaDirectory="chromaDB", faissDirectory="faissIndex"
):
    """Selects which vector database to be used and iniciates said.
    When given a path to a document folder, creates embeddings for the documents and stores them in the db.
    
    Args:

    pathToDocs (str): The path to the folder with documents
    useChroma (bool): Whether to use chroma or not
    chromaDirectory (str): The path of the folder to store the chroma vector db
    faissDirectory (str): The path of the folder to store the faiss vector db

    Returns:

    embedding: The embeddings
    dbEngine: The vector db package being used
    
    """

    # Using OpenAI embeddings
    embedding = OpenAIEmbeddings()

    # Selects vectorstore based on paramater
    if useChroma:
        from langchain.vectorstores import Chroma as dbEngine
    else:
        from langchain.vectorstores import FAISS as dbEngine

    # Finds file to embbed and store in vectorstore
    if pathToDocs == "":
        db = dbEngine.from_texts([""], embedding)
    else:
        documents = []
        for file in os.listdir(pathToDocs):
            fileLower = file.lower()
            if fileLower.endswith(".txt"):
                text_path = f"{pathToDocs}/{file}"
                loader = TextLoader(text_path, encoding="utf8")
                documents.extend(loader.load())
            elif fileLower.endswith(".pdf"):
                pdf_path = f"{pathToDocs}/{file}"
                loader = PyPDFLoader(pdf_path)
                documents.extend(loader.load())
        
        # Creates vectorstore and persists it
        if useChroma:
            db = dbEngine.from_documents(documents, embedding, persist_directory=chromaDirectory)
            db.persist()
        else:
            db = dbEngine.from_documents(documents, embedding)
            db.save_local(faissDirectory)


    return dbEngine



def getRetriever(embedding, dbEngine, useChroma: bool = False, chromaDirectory="chromaDB", faissDirectory="faissIndex"
):
    """Returns a "VectorStoreRetriever" when pointed to a vector database.
    By default, returns an "empty" retriever.

    Args:

    embedding: Which embeddings should be used for the retriever
    dbEngine: The vector database being used
    useChroma (bool): Whether to use chroma or not
    chromaDirectory (str): The path of the folder to store the chroma vector db
    faissDirectory (str): The path of the folder to store the faiss vector db
    (dbEngine, useChroma, chromaDirectory and faissDirectory must be the same as the dbEmbedding function params.)

    Returns:

    VectorStoreRetriever: A retriever
    """

    # Selects vectorstore for retriever
    if useChroma:
        db = dbEngine(persist_directory=chromaDirectory,
                embedding_function=embedding)
    else:
        db = dbEngine.load_local(faissDirectory, embedding)

    # Using vectorstore as a retriever to retrieve k documents
    retriever = db.as_retriever(
        search_type="similarity",
        search_kwargs={"k": 8},
    )

    return retriever


def queryVectorDatabase(query: str, retriever) -> str:
    """Takes a "VectorStoreRetriever" and a query (string) and returns an answer.
    Mostly based on LangChain's RetrievalQA chain.

    Args:

    query (str): The query to search for
    retriever (VectorStoreRetriever): The retriever to search in

    Returns:

    str: The answer
    """

    # Using RetrievalQA from Langchain. Uses ChatGPT model to return response to query using documents from retriever
    qa = RetrievalQA.from_chain_type(
        ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0),
        chain_type="stuff",
        retriever=retriever,
        return_source_documents=True,
    )
    r = qa({"query": query})

    answer = r["result"]

    out = ""
    if answer:
        out += f"{answer}"
    return out