import random
import time

import rclpy
from rclpy.node import Node

import speech_recognition as sr

from trajectory_interfaces.srv import UserInput

def recognize_speech_from_mic(recognizer, microphone):
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        response["error"] = "Unable to recognize speech"

    return response

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(UserInput, 'input', self.user_input_callback)

    def user_input_callback(self, request, response):
        SHOWN_WORDS = ["red", "blue", "yellow", "green"]
        WORDS = ["red", "blue", "yellow", "green", "matt"]
        PROMPT_LIMIT = 5

        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        instructions = (f"Choose the following target colour:\n {SHOWN_WORDS}\n")

        print(instructions)
        time.sleep(1)
        for j in range(PROMPT_LIMIT):
            print('Speak!')
            guess = recognize_speech_from_mic(recognizer, microphone)
            if guess["transcription"]:
                if guess["transcription"].lower() not in WORDS:
                    print("Sorry, you can't choose that as a target.\n")
                    continue
                else:
                    break
            if not guess["success"]:
                break
            print("I didn't catch that. What did you say?\n")
        final_guess = guess['transcription']
        print(f"You said: {final_guess}\n")

        response.answer = final_guess
        return response


def entry_point(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)

