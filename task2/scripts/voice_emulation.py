#!/usr/bin/python3

import roslib
import time
import rospy
import speech_recognition as sr
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


class SpeechDialog:
	def __init__(self):
		rospy.init_node('speech_dialog', anonymous=True)
		self.sr = sr.Recognizer()
		self.mic = sr.Microphone()
		self.sc = SoundClient()
		self.colors = ['red', 'green', 'blue', 'yellow', 'black']
		
		self.start_sub = rospy.Subscriber('/start_dialog', String, self.talk)
		self.result_pub = rospy.Publisher('/dialog_results', String, queue_size=10)
		
		rospy.sleep(0.25)
		
	def talk(self, data):
		self.greet()
		self.recognize_speech()
	
	def greet(self):
		sound = self.sc.say("Hello, do you know where the robber is hiding?")
		rospy.sleep(1.5)
			
			
	def goodbye(self):
		sound = self.sc.say("Okay, thank you, goodbye.")
		
	def sad_goodbye(self):
		sound = self.sc.say("Anluko, goodbye.")

	def recognize_speech(self):
		with self.mic as source:
			print('Adjusting mic for ambient noise...')
			self.sr.adjust_for_ambient_noise(source)
			print('Listening now!')
			audio = self.sr.listen(source)
            	
		print('I am now processing the sounds you made.')
		recognized_text = ''
		try:
			recognized_text = self.sr.recognize_google(audio, language='en-US')
		except sr.RequestError as e:
			print('API is probably unavailable', e)
		except sr.UnknownValueError:
			print('Did not manage to recognize anything.')
		

		found_colors = ""
		found_color = False
		for color in self.colors:
			if color in recognized_text.lower():
				print(recognized_text)
				found_colors += color + ', '
				print('Maybe on the ' + color + ' cylinder.')
				found_color = True

		
		
		if found_color:
			self.result_pub.publish(found_colors)
			self.goodbye()
		else:
			self.sad_goodbye()
			print('This person does not know')
			self.result_pub.publish('none')
		
		
if __name__ == '__main__':
	sd = SpeechDialog()
	rospy.spin()