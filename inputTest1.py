from inputs import devices
from inputs import get_gamepad
from time import sleep

#for device in devices:
#	print(device)
i = 0

while 1:
	events = get_gamepad()
	for event in events:
#		print(event.ev_type, event.code, event.state)
		if event.code == 'ABS_X':
			if event.state > 0:
				print("right",event.state)
			if event.state <0:
				print("left",event.state)

		if event.code == 'ABS_Y':
			if event.state > 0:
				print("up",event.state)
			if event.state <0:
				print("down",event.state)

		if event.code == 'RightTrigger':
			if event.state == 1:
				print("up")

	#print(i)
	sleep(0.01)
	i = i+1
#classic
#kp = input("WASD to move")
#print(kp)