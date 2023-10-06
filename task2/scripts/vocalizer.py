#!/usr/bin/python3

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class Vocalizer:
    def __init__(self) -> None:
        rospy.init_node('vocalizer')
        self.image_sub = rospy.Subscriber("/vocalizer_req", String, self.vocalizer_callback)
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0


    def vocalizer_callback(self, msg):
        stri = msg.data
        print("Vocalizer callback: ", stri)
        self.soundhandle.say(stri, self.voice, self.volume)


def main():

    vocalizer = Vocalizer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
