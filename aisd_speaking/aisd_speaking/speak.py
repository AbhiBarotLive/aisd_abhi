import io
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
from aisd_interfaces.srv import Speak

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    #here the speak_callback function will request the words in english default set english language
    def speak_callback(self, request, response):
        with io.BytesIO() as f:                                        #here bytesIO is used for binary data
            gTTS(text=request.words, lang='en').write_to_fp(f)         #here gTTs is googl's text-to-speech API
                                                                       #It will write spoken mp3 data into a file
            f.seek(0)
            song = AudioSegment.from_file(f, format="mp3")             #It will play song or we can say speak the songs/words of songs
            play(song)
            response.response = "OK"
            return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
