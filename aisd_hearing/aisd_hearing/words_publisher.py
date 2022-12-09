#!/usr/bin/env python

#importing necesseary packages

from std_msgs.msg import String
from aisd_interfaces.srv import Speak
import rclpy
from rclpy.node import Node

import wave
import struct

import sys
import glob
import os

from deepspeech import Model

#creating WordsPublisher class which inheerits from node.

class WordsPublisher(Node): 
    def __init__(self):
        super().__init__('words_publisher')       #init file will calls the node class's constructor and gives a node name words_publisher
        self.publisher_ = self.create_publisher(String, 'words', 10)
        self.subscription = self.create_subscription(
            String,
            'recording',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.model = None
        ##################################
        #(not required for Assignment 3) Change the model_path to a parameter
        # Change this model_path (to where your model file is stored) to use your ros2 workspace,
        # and your GitHub repository name.  Don't forget to put the model file
        # in this directory.

        #model_path = "/home/aisd/ros2_ws/src/aisd-tgkell/aisd_hearing/model"
        model_path = "/home/aisd/ros2_ws/src/aisd-AbhiBarotLive/aisd_hearing/model"
        ##################################
        if model_path != None:
            self.load_model(model_path)
        #self.commands = commands
        #self.dictionary = dictionary

    #it will get the audio input
    def listener_callback(self, msg):
        self.get_logger().info('analysing: "%s"' % msg.data)
        audio_file = wave.open(msg.data)
        fs = audio_file.getframerate()
        audio_string = audio_file.readframes(-1)
        audio = [struct.unpack("<h", audio_string[i:i+2])[0]
                 for i in range(0, len(audio_string), 2)]

        text = String()
        text.data = self.stt(fs, audio)
        self.get_logger().info('I heard: "%s"' % text.data)
        self.publisher_.publish(text)


    # Default values for n_feaures, n_context, beam_width are from
    # github.com/mozilla/DeepSpeech/blob/master/native_client/python/client.py
    def load_model(self, model_path, n_features=26,n_context=9,beam_width=500):
        model_path += "/"
        model_file = model_path + "deepspeech-0.9.3-models.pbmm"
        self.model = Model(model_file)

    def stt(self,fs, audio):
        assert self.model != None, "a model must be loaded before testing"
        transcription = self.model.stt(audio)

        return transcription

#The main function defined which will call the the class

def main(args=None):
    rclpy.init(args=args)

    words_publisher = WordsPublisher()

    rclpy.spin(words_publisher)

    words_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
