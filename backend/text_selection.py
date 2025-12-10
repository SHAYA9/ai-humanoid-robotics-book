from . import neon_client, gemini_client, qdrant_client

async def process_selection(question: str, context: str):
    """
    Core logic to handle a user's question about a selected piece of text.
    """
    # 1. Search Qdrant for relevant passages to augment the context
    search_results = await qdrant_client.search_for_passages(context)

    # 2. Construct the prompt for the Gemini LLM
    context_passages = [hit.payload['text'] for hit in search_results]
    sources = ", ".join(list(set(f"{hit.payload['module']}/{hit.payload['page']}" for hit in search_results)))

    prompt = f"""
        You are an expert AI assistant for the 'AI Humanoid Robotics Book'.
        A user has selected the following text from the book:
        ---
        {context}
        ---

        Here are the most relevant passages from the book to provide additional context:
        ---
        {"\n---\n".join(context_passages)}
        ---

        Based ONLY on the user's selected text and the provided context passages,
        answer the user's question concisely. Do not use any external knowledge.
        If the answer is not in the provided text, state that clearly.

        Question: {question}
    """

    # 3. Call the Gemini API to get the answer
    answer = await gemini_client.generate_answer(prompt)

    # 4. Save the interaction to the chat history in Neon DB
    await save_interaction(question, answer, sources)

    return {"answer": answer, "sources": sources}

async def save_interaction(question, answer, sources):
    """Saves the chat interaction to the Neon Postgres database."""
    conn = await neon_client.get_db_connection()
    if not conn:
        print("Could not save interaction, no DB connection.")
        return

    try:
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO chat_history (question, answer, sources) VALUES (%s, %s, %s)",
                (question, answer, sources)
            )
        conn.commit()
    except Exception as e:
        print(f"Error saving chat history: {e}")
    finally:
        if conn:
            conn.close()
