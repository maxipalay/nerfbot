"""
UserInput node that could help with audio user input.

Services
--------
    input [UserInput]: user input.

Returns
-------
    None

"""
import random
import time

import rclpy
from rclpy.node import Node

import speech_recognition as sr

from trajectory_interfaces.srv import UserInput

# Start Citation [1]
def recognize_speech_from_mic(recognizer, microphone):
    """Recognize speech from a microphone.

    Args:
        recognizer (SpeechRecognizer): An instance of the speech recognition Recognizer class.
        microphone (Microphone): An instance of the speech recognition Microphone class.

    Raises:
        TypeError: If `recognizer` is not an instance of Recognizer.
        TypeError: If `microphone` is not an instance of Microphone.

    Returns:
        dict: A dictionary containing recognition results.
            - 'success' (bool): True if recognition was successful, False otherwise.
            - 'error' (str): An error message if an error occurred during recognition.
            - 'transcription' (str): The recognized speech transcription if successful, None otherwise.
    """
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
# End Citation [1]

class UserInput(Node):
    """
    A UserInput node that could help with audio user input

    Args:
    ----
        Node (ros node): a node's superclass

    """
    def __init__(self):
        super().__init__('user_input')
        self.srv = self.create_service(UserInput, 'input', self.user_input_callback)

    def user_input_callback(self, request, response):
        """A callback function for processing user input.

        Args:
            request (UserInput): An instance of the UserInput message representing the user's request.
            response (UserInput.Response): An instance of the UserInput response message.

        Returns:
            UserInput.Response: The processed user input stored in the response message.
        """
        SHOWN_WORDS = ["red", "blue", "yellow", "green"]
        WORDS = ["red", "blue", "yellow", "green", "matt"]
        PROMPT_LIMIT = 5

        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        final_guess = None
        instructions = (f"Choose the following target colour:\n {SHOWN_WORDS}\n")

        self.get_logger().info(f"{instructions}")
        time.sleep(1)
        for j in range(PROMPT_LIMIT):
            self.get_logger().info(f"Speak!")
            guess = recognize_speech_from_mic(recognizer, microphone)
            if guess["transcription"]:
                if guess["transcription"].lower() not in WORDS:
                    self.get_logger().info("Sorry, you can't choose that as a target.\n")
                    continue
                else:
                    break
            if not guess["success"]:
                break
            self.get_logger().info("I didn't catch that. What did you say?\n")
        final_guess = guess['transcription']
        self.get_logger().info(f"You said: {final_guess}\n")

        response.answer = final_guess
        return response


def entry_point(args=None):
    rclpy.init(args=args)
    user_input = UserInput()
    rclpy.spin(user_input)

