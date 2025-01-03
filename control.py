import cv2
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from time import sleep
import time
import numpy as np

GPIO.setmode(GPIO.BCM)

class Motor():
    def __init__(self,Ena,In1,In2):
        self.Ena = Ena
        self.In1 = In1
        self.In2 = In2
        GPIO.setup(self.Ena,GPIO.OUT)
        GPIO.setup(self.In1,GPIO.OUT)
        GPIO.setup(self.In2,GPIO.OUT)                                                               
        self.pwm = GPIO.PWM(self.Ena,100) # operates at 100 Hz (check?)
        self.pwm.start(0)

    def moveF(self,x=100,t=0): # by default, it operates at 50% speed
        GPIO.output(self.In1,GPIO.LOW)
        GPIO.output(self.In2,GPIO.HIGH)
        self.pwm.ChangeDutyCycle(x) # x% of the speed
        sleep(t)
    
    def moveB(self,x=100,t=0): # by default, it operates at 50% speed
        GPIO.output(self.In1,GPIO.HIGH)
        GPIO.output(self.In2,GPIO.LOW)
        self.pwm.ChangeDutyCycle(x) # x% of the speed
        sleep(t)
    
    def stop(self,t=0):
        GPIO.output(self.In1,GPIO.LOW)
        GPIO.output(self.In2,GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        sleep(t)


class Servo:
    
    def __init__(self, pwm_pin, frequency=50):
        # Initialize GPIO settings
        #GPIO.setmode(GPIO.BCM)
        self.pwm_pin = pwm_pin
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        
        # Initialize PWM settings
        self.pwm = GPIO.PWM(self.pwm_pin, frequency)
        self.pwm.start(0)
    
    def set_pwm(self,pwm):
        GPIO.output(self.pwm_pin, True)
        self.pwm.ChangeDutyCycle(pwm)
        sleep(1)
        
    def cleanup(self):
        # Stop PWM and clean up GPIO
        self.pwm.stop()
        GPIO.cleanup()
        
class DistanceSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        # Setup pins
#         GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def get_distance(self):
        # Ensure trigger pin is low
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(0.000002)
        
        # Create a 10us pulse on the trigger pin
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)
        
        # Listen for echo
        while GPIO.input(self.echo_pin) == 0:
            pass
        echo_start_time = time.time()
        while GPIO.input(self.echo_pin) == 1:
            pass
        
        echo_stop_time = time.time()
        
        # Calculate distance
        ping_travel_time = echo_stop_time - echo_start_time
        echo_travel_distance = ping_travel_time * 34300
        distance = echo_travel_distance / 2
        return round(distance, 1)

    def continuous_detection(self, delay=0.1):
        try:
            while True:
                distance = self.get_distance()
                print(f'Distance: {distance} cm')
                time.sleep(delay)
        except KeyboardInterrupt:
            GPIO.cleanup()
            print('GPIO CleanCed up and Good to Go')

class Car():
    def __init__(self, motor_left_pins, motor_right_pins,sensor_left_pins,left_servo,right_servo):
        # motor_left_pins and motor_right_pins should be tuples containing the GPIO pin numbers (Ena, In1, In2)
        self.motor_left = Motor(motor_left_pins[0], motor_left_pins[1], motor_left_pins[2])
        self.motor_right = Motor(motor_right_pins[0], motor_right_pins[1], motor_right_pins[2])
        self.sensor_left = DistanceSensor(sensor_left_pins[0], sensor_left_pins[1])
        #self.sensor_front = DistanceSensor(sensor_front_pins[0], sensor_front_pins[1])
        self.servo_left = Servo(left_servo,50)
        self.servo_right = Servo(right_servo,50)

    def forward(self, speed=100, duration=0):
        self.motor_left.moveF(speed)
        self.motor_right.moveF(speed)
        if duration > 0:
            sleep(duration)
            self.stop()

    def backward(self, speed=100, duration=0):
        self.motor_right.moveB(speed)
        self.motor_left.moveB(speed)
        if duration > 0:
            sleep(duration)
            self.stop()

    def turn_left(self, speed=100, duration=0):
        self.motor_left.moveB(speed)  # Stop the left motor
        self.motor_right.moveF(speed)  # Move the right motor forward
        if duration > 0:
            sleep(duration)
            self.stop()

    def turn_right(self, speed=100, duration=0):
        self.motor_right.moveB(speed)  # Stop the right motor
        self.motor_left.moveF(speed)  # Move the left motor forward
        if duration > 0:
            sleep(duration)
            self.stop()

    def stop(self, duration=0):
        self.motor_left.stop(duration)
        self.motor_right.stop(duration)
        if duration > 0:
            sleep(duration)
    def auto_straight(self,dis):
    # front_dis = self.sensor_front.get_distance()
        left_dis = self.sensor_left.get_distance()
        sleep(0.3)
        print(f"left_dis:{left_dis}")
        if(left_dis-dis>1):
            self.motor_left.moveF(0)
            self.motor_right.moveF(0)
            print("left")
        elif(left_dis-dis<-1):
            self.motor_left.moveF(0)
            self.motor_right.moveF(0)
            print("right")
        else:
            self.motor_left.moveF(100)
            self.motor_right.moveF(100)
            print("front")
        sleep(0.2)

    def Wheels_open(self):
        self.servo_left.set_pwm(5)
        self.servo_right.set_pwm(3)
    
    def Wheels_closed(self):
        self.servo_left.set_pwm(3)
        self.servo_right.set_pwm(5)
    
#     def get_right(self):
#         return self.sensor_front.get_distance()
#     def get_left(self):
#         return self.sensor_left.get_distance()
        
        
def main():
    #motor_left_pins = (16, 20, 21)
    # Define the GPIO pins for the motors, sensors, and servos
    motor_left_pins = (16, 20, 21)  # Example GPIO pin numbers for the left motor (Ena, In1, In2)
    motor_right_pins = (22, 5, 6)  # Example GPIO pin numbers for the right motor (Ena, In1, In2)
    
    sensor_left_pins = (19, 26)  # Example GPIO pin numbers for the left distance sensor (Trig, Echo)
    #sensor_front_pins = (24, 23)
    
    left_servo_pin = 17  # GPIO pin for the left servo
    right_servo_pin = 27  # GPIO pin for the right servo
    sensor = 24
    sensor_2 = 25
    GPIO.setup(sensor, GPIO.IN)#sensor related
    GPIO.setup(sensor_2, GPIO.IN)
    # Initialize the Car
    my_car = Car(motor_left_pins, motor_right_pins, sensor_left_pins, left_servo_pin, right_servo_pin)
    3
    my_car.stop(duration = 0.5)
    buttom_pin = 23
    GPIO.setup(buttom_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    kn=0
    
    try:
        while True:
            EXIT = False
            print("Loop")
            kn=kn+1
            print(kn)
            if (GPIO.input(buttom_pin) == GPIO.HIGH):
                print("1")
                while(GPIO.input(buttom_pin) == GPIO.HIGH):
                    pass
                distance = GPIO.input(sensor)
                print(f"Dis: {distance} cm")
                count=0
                while (distance == 1):
                    #distance = GPIO.input(sensor)
                    #print(f"Dis: {distance} cm")
                    #my_car.auto_straight(left_dis_init)
                    distance = GPIO.input(sensor)
                    my_car.forward(speed=100, duration =0.5)
                    count=count+0.5
                    print(distance)
                    print(count)
                    if (GPIO.input(buttom_pin) == GPIO.HIGH):
                        EXIT = True
                        break
        #            my_car.forward(speed = 100, duration=0.5)
                if(EXIT==True):
                    break
                print("count=")
                print(count)
                mode=0
                if(count<18):
                    mode=1
                elif(count>33):
                    mode=3
                else:
                    mode=2
                print("Opening Wheels...")
                sleep(2)
                my_car.stop(duration = 0.2)
                my_car.Wheels_open()
                sleep(3)
                 
                 #Test basic movements
#                 print("Moving forward...")
                #my_car.forward(speed=100, duration=40)
                
                distance_2 = GPIO.input(sensor_2)
                while (distance_2 == 1):
                    distance_2 = GPIO.input(sensor_2)
                    print(f"Dis: {distance_2} cm")
                    my_car.forward(speed=100, duration = 0.2)
                    if (GPIO.input(buttom_pin) == GPIO.HIGH):
                        EXIT=True
                        break
                if(EXIT==True):
                    break
                my_car.Wheels_closed()
                if(GPIO.input(buttom_pin) == GPIO.HIGH):
                    break
                sleep(2)
                # Test sensor reading
                #print("Front sensor distance: ", my_car.sensor_front.get_distance())
                direction = 0
                picam2 = Picamera2()
                dispW=1280
                dispH=720
                picam2.preview_configuration.main.size = (dispW,dispH)
                picam2.preview_configuration.main.format = "RGB888"
                picam2.preview_configuration.controls.FrameRate=30
                picam2.preview_configuration.align()
                picam2.configure("preview")
                picam2.start()

                # define variables
                fps=0
                pos=(30,60)
                font=cv2.FONT_HERSHEY_SIMPLEX
                height=1.5
                weight=3
                myColor=(0,0,255)
                GhueLow = 51
                GhueHigh = 91
                GsatLow = 43
                GsatHigh = 168
                GvalLow = 45
                GvalHigh = 236
                RhueLow = 168
                RhueHigh = 179
                RsatLow = 150
                RsatHigh = 255
                RvalLow = 0
                RvalHigh = 255

                while direction == 0:
                    if(GPIO.input(buttom_pin) == GPIO.HIGH):
                        EXIT=True
                        break
        #             direction = camera_sensor.process_frame()
                    tStart=time.time()
                    
                    # grab a frame from Pi camera
                    frame= picam2.capture_array()
                    
                    # if you need to invert the picture, uncomment the following line
                    #frame=cv2.flip(frame,-1)
                    
                    # transformation of color encoding from BGR (not RGB) to HSV (easier to adjust by hand)
                    frameHSV=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                    
                    # add FPS text
                    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)

                    
                    # define my masks
                    GlowerBound=np.array([GhueLow,GsatLow,GvalLow])
                    GupperBound=np.array([GhueHigh,GsatHigh,GvalHigh])
                    GmyMask=cv2.inRange(frameHSV,GlowerBound,GupperBound)
                    
                    RlowerBound=np.array([RhueLow,RsatLow,RvalLow])
                    RupperBound=np.array([RhueHigh,RsatHigh,RvalHigh])
                    RmyMask=cv2.inRange(frameHSV,RlowerBound,RupperBound)
                    
                    # define my objects
                    GmyObject=cv2.bitwise_and(frame,frame, mask=GmyMask)
                    RmyObject=cv2.bitwise_and(frame,frame, mask=RmyMask)
                    
                    # get the contour
                    Gcontours,junk=cv2.findContours(GmyMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    Rcontours,junk=cv2.findContours(RmyMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    if len(Gcontours)>0:
                        # sort all contours that satisfy our HSV requirement
                        Gcontours=sorted(Gcontours,key=lambda x:cv2.contourArea(x),reverse=True)
                        
                        # get the largest contour
                        Gcontour=Gcontours[0]
                        
                        # define a rectangle that surrounds the largest contour
                        x1,y1,w1,h1=cv2.boundingRect(Gcontour)
                        cv2.rectangle(frame,(x1,y1),(x1+w1,y1+h1),(0,255,0),3)
                    
                        if len(Rcontours)>0:
                            # sort all contours that satisfy our HSV requirement
                            Rcontours=sorted(Rcontours,key=lambda x:cv2.contourArea(x),reverse=True)
                            
                            # get the largest contour
                            Rcontour=Rcontours[0]
                            
                            # define a rectangle that surrounds the largest contour
                            x2,y2,w2,h2=cv2.boundingRect(Rcontour)
                            cv2.rectangle(frame,(x2,y2),(x2+w2,y2+h2),(0,0,255),3)
                            if x2+w2/2>x1+w1/2:
                                print('Green is on the left')
                                direction =1
                            if x2+w2/2<x1+w1/2:
                                print('Green is on the right')
                                direction =2
                    print()
                    cv2.imshow("Camera", frame)
                    
                    # exit the program when key 'q' is pressed
                    if cv2.waitKey(1)==ord('q'):
                        break
                    
                    # stop timing
                    tEnd=time.time()
                    
                    # calculate frames per second
                    loopTime=tEnd-tStart
                    fps=.9*fps + .1*(1/loopTime)

                    print("direction value is:")
                    print(direction)
                    my_car.forward()# moving
                cv2.destroyAllWindows()
                
                if(EXIT==True):
                    break
                sleep(15)

                if(direction == 1):
                    my_car.turn_left(speed=100,duration=6.3)
                    print("turing left")
                    #try setting parameters here
#                     mode=3
                    if(mode==1):
                        my_car.forward(speed =100, duration = 51)
                    elif(mode==2):
                        my_car.forward(speed =100, duration = 43)
                    else:
                        my_car.forward(speed =100, duration = 34)
                    my_car.turn_left(speed=100,duration=6)
                    print("turing left")
                    my_car.forward(speed =100, duration = 22)
                elif(direction == 2):
                    my_car.turn_right(speed=100,duration=6.9)
                    print("turning right")
                    #try setting parameters here
                    #mode=1
                    if(mode==1):
                        my_car.forward(speed =100, duration = 53)
                        my_car.turn_right(speed=100,duration=6.6)
                    elif(mode==2):
                        my_car.forward(speed =100, duration = 44)
                        my_car.turn_right(speed=100,duration=6.7)
                    else:
                        my_car.forward(speed =100, duration = 34)
                        my_car.turn_right(speed=100,duration=6.9)
                    print("turning right")
                    my_car.forward(speed =100, duration = 26)
                my_car.stop(duration=0.5)
                break
            sleep(0.5)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print('GPIO Good to Go')
    finally:
            # Cleanup GPIO settings before exiting
        GPIO.cleanup()

if __name__ == "__main__":
    main()
