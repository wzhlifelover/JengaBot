import RPi.GPIO as GPIO
from time import sleep

class pump:
    def __init__(self,dir1,dir2,pwm_pin):
        self.dir1 = dir1
        self.dir2 = dir2
        self.pwm_pin = pwm_pin

    def activate(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir1,GPIO.OUT)
        GPIO.output(self.dir1,GPIO.HIGH)
        GPIO.setup(self.dir2,GPIO.OUT)
        GPIO.output(self.dir2,GPIO.LOW)
        GPIO.setup(self.pwm_pin,GPIO.OUT)
        #CREATE A PWM OBJECT
        self.p = GPIO.PWM(self.pwm_pin,1000)
        self.p.start(0)
        
    def deactivate(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir1,GPIO.OUT)
        GPIO.setup(self.dir2,GPIO.OUT)
        GPIO.output(self.dir1,GPIO.LOW)
        GPIO.output(self.dir2,GPIO.LOW)
        GPIO.cleanup()
        
    def pwm_value(self,pwm):
        self.p.start(pwm)

pump1 = pump(5,6,13)
pump1.activate()
pump1.pwm_value(25)
sleep(2)
pump1.pwm_value(50)
sleep(2)
pump1.pwm_value(100)
sleep(5)
pump1.pwm_value(0)
pump1.deactivate()