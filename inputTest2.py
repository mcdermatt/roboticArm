import pygame as pygame
import time

pygame.init()
pygame.joystick.init()

stick = pygame.joystick.Joystick(0)
stick.init()

axisNum = stick.get_numaxes()
buttonNum = stick.get_numbuttons()
print(axisNum)
print(buttonNum)

while pygame.joystick.get_count()>0:
    pygame.event.get()
    stick_L = (stick.get_axis(0),stick.get_axis(1))
    print(stick_L)
    stick_R = (stick.get_axis(3),stick.get_axis(4))
#    pads = (stick.get_axis(2),stick.get_axis(5))
    for button in range(buttonNum):
        #print(button, stick.get_button(button))
        time.sleep(0.1)
    #time.sleep()



pygame.joystick.quit()