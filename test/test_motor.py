from time import sleep

import RPi.GPIO as GPIO

relayPin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(relayPin,GPIO.OUT)

try:  
    # here you put your main loop or block of code  
    while True:  
        GPIO.output(relayPin, GPIO.HIGH) 
        sleep(0.1) 
        GPIO.output(relayPin, GPIO.LOW)
        sleep(0.1) 
  
except KeyboardInterrupt:  
    # here you put any code you want to run before the program   
    # exits when you press CTRL+C  
    print("interuptted")
  
except:  
    # this catches ALL other exceptions including errors.  
    # You won't get any error messages for debugging  
    # so only use it once your code is working  
    print("error")
  
finally:  
    GPIO.cleanup() # this ensures a clean exit