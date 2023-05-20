import requests
from random import randint
from time import sleep

while True:
	throttle = input("Enter throttle: ")
	roll = input("Enter roll: ")
	pitch = input("Enter pitch: ")
	yaw = input("Enter yaw: ")

	# throttle, roll, pitch, yaw = randint(1, 255), randint(1, 255), randint(1, 255), randint(1, 255)
	command = {'throttle': throttle, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
	print(command)

	sleep(0.1)

	response = requests.post("http://192.168.4.1/command", params=command)
	# print("Response: ", response.text)