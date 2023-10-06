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
		self.negations=['not',"isn't","don't"]

		
		self.start_sub = rospy.Subscriber('/start_dialog', String, self.talk)
		self.found_sub = rospy.Subscriber("/vocalizer_req", String, self.vocalizer_callback) # merging
		self.possible_cylinder_colors_pub = rospy.Publisher('/possible_cylinders',String, queue_size=10)
		self.impossible_cylinder_colors_pub = rospy.Publisher('/impossible_cylinders', String, queue_size=10)

		self.voice = 'voice_kal_diphone'
		self.volume = 1.0
		
		rospy.sleep(0.25)

	def vocalizer_callback(self, msg):
		stri = msg.data
		print("Vocalizer callback: ", stri)
		self.sc.say(stri, self.voice, self.volume)
		
	def talk(self, data):
		self.greet()
		self.recognize_speech()
	
	def greet(self):
		rospy.sleep(1.5)
		sound = self.sc.say("Hello! Do you know where the most wanted robber is hiding?")
		rospy.sleep(1.5)
			
			
	def goodbye(self):
		sound = self.sc.say("Thank you for your cooperation. Goodbye!")
		
	def sad_goodbye(self):
		sound = self.sc.say("That's unfortunate, goodbye.")

	def recognize_speech(self):
		with self.mic as source:
			print('Adjusting mic for ambient noise...')
			self.sr.adjust_for_ambient_noise(source)
			print('Start speaking!')
			audio = self.sr.listen(source)
            	
		print('Processing sound...')
		recognized_text = ''
		try:
			recognized_text = self.sr.recognize_google(audio, language='en-US')
			print(recognized_text)
		except sr.RequestError as e:
			print('API is probably unavailable', e)
		except sr.UnknownValueError:
			print('Did not manage to recognize anything.')
		

		found_colors = ""

		valid_colors=[]
		invalid_colors=[]

		words = recognized_text.lower().split()
		found_color = False
		negation_found=False

		
		for word in words:
			if word in self.negations:
				negation_found = True
			elif negation_found:
				for color in self.colors:
					if color == word:
						invalid_colors.append(color)
						print("Invalid color found: ", color)
						negation_found  = False
						found_color = True
						break
			else:
				for color in self.colors:
					if color == word:
						valid_colors.append(color)
						print("Valid color found: ", color)
						found_color = True
						break
                
		
		if found_color:
			#adding the array into a string so that it can be published
			#possible=', '.join(valid_colors)
			#impossible=', '.join(invalid_colors)

			#publishig results
			self.possible_cylinder_colors_pub.publish(', '.join(valid_colors))
			self.impossible_cylinder_colors_pub.publish(', '.join(invalid_colors))
			print("Final invalid colors: ",invalid_colors)
			print("Final valid colors: ",valid_colors)
			self.goodbye()
		else:
			self.sad_goodbye()
			print('No important information recieved.')
			#
			self.impossible_cylinder_colors_pub.publish('')
			self.possible_cylinder_colors_pub.publish('')

			#self.result_pub.publish('none')
		
		
if __name__ == '__main__':
	sd = SpeechDialog()
	rospy.spin()