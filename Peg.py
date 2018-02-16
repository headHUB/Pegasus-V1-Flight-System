#!/usr/bin/env python

import sys
import time
import pygame
import serial


EXIT = False                          # set exit to false

pygame.init() #initiaise pygame
controls = pygame.joystick.Joystick(0) # call the joystic controls
clock = pygame.time.Clock()           # intialise pygame refresh and call it clock
controls.init()                        # initialise the controls

arduino =  serial.Serial('/dev/ttyUSB1', 9600,timeout = 1) #connect to the arduino's serial port
time.sleep(2)

while not EXIT:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            EXIT = True
        
    old_value1 = controls.get_axis(0)  # X-axis
    old_value2 = controls.get_axis(1)  # Y-axis
    old_value3 = controls.get_axis(3)  # Rudder   ,,,,,,,,,,,
    old_min = -1
    old_max = 1
    new_min = 100
    new_max = 355
                                        # take original values (-1 to 1) and map it to a new range btween 100 and 355***
    new_value1 = ( ( old_value1 - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
    new_value2 = ( ( old_value2 - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
    new_value3 = ( ( old_value3 - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min

    a = str(int(round(new_value1)))   
    b = str(int(round(new_value2)))  # converts values from floats to integers and then to strings
    c = str(int(round(new_value3)))
    h = '000'

    for x in range(0,11):
        if controls.get_button(x) == 1:
            h = '0' + str(x+1)
            if x < 9:
                h = '00' + str(x+1)
                 
    control = ['<' + a,b,c,h + '>']    # save strings in a list
    cstring = ",".join(control)     # convert list to a single string with commas seperating values
        
    print(cstring)
    arduino.write(cstring)          # print string to shell and write data to arduino with a 100ms delay
    time.sleep(0.0001)
        
    if controls.get_button(11) == 1:
        pygame.quit()              # emergency shut down
        quit()
        
    clock.tick(1000)                  # refresh at 1000 fps
            
pygame.quit()
quit()                             ## end program if while loop is broken
close()
