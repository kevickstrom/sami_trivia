#This is the initial commit

import speech_recognition as sr
import OpenAI
import gTTS
import playsound
import tempfile
import os
'''
from openai
from gtts
from playsound
'''

client = OpenAI(api_key="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")  # <--- Put your real OpenAI key here

trivia_question = "What is the capital of France?"
correct_answer = "Paris"

def speak(text):
    print(f"SAMI says: {text}")
    tts = gTTS(text=text, lang='en')
    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
        tts.save(fp.name)
        playsound(fp.name)
        os.remove(fp.name)

def listen_and_transcribe():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        speak("Press Enter when you're ready to record your answer.")
        input("Press Enter to start recording...")
        speak("Listening. Speak now.")
        audio = recognizer.listen(source)

    try:
        text = recognizer.recognize_google(audio)
        speak(f"You said: {text}")
        return text
    except sr.UnknownValueError:
        speak("Sorry, I didn't catch that.")
        return "Could not understand audio"
    except sr.RequestError as e:
        speak("Speech service error occurred.")
        return f"Error: {e}"

def check_answer(user_answer):
    prompt = (
        f"The trivia question is: '{trivia_question}'. "
        f"The correct answer is '{correct_answer}'. "
        f"The user answered: '{user_answer}'. "
        f"Is the user's answer correct? Reply with only 'Correct' or 'Incorrect'."
    )

    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are an assistant that checks trivia answers."},
            {"role": "user", "content": prompt}
        ]
    )

    result = response.choices[0].message.content.strip()
    speak(f"That is {result}.")
    return result

def main():
    speak("Welcome to the trivia game.")
    speak(f"Here is your question: {trivia_question}")
    print("Question:", trivia_question)
    user_response = listen_and_transcribe()
    check_answer(user_response)

if __name__ == "__main__":
    main()
