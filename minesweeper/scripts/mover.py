import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

def callback(data):
    if data.linear.x > 0:
         GPIO.output(35, GPIO.HIGH)
         GPIO.output(36, GPIO.HIGH)
         pwm1.start(100)			# set speed for M1 at 100%
         pwm2.start(100)
         #print("f")
    elif data.linear.x < 0:
         GPIO.output(35, GPIO.LOW)
         GPIO.output(36, GPIO.LOW)
         pwm1.start(100)			# set speed for M1 at 100%
         pwm2.start(100)
         #print("b")
    elif data.angular.z > 0:
         GPIO.output(35, GPIO.LOW)
         GPIO.output(36, GPIO.HIGH)
         pwm1.start(100)			
         pwm2.start(100)
         #print("l")
    elif data.angular.z < 0:
         GPIO.output(35, GPIO.HIGH)
         GPIO.output(36, GPIO.LOW)
         pwm1.start(100)			
         pwm2.start(100)
         #print("r")
    else:
         GPIO.output(35, GPIO.LOW)
         GPIO.output(36, GPIO.LOW)
         pwm1.stop()			
         pwm2.stop()
         #print("s")



def listener():
    rospy.init_node("listener",anonymous=True)
    rospy.Subscriber("cmd_vel",Twist,callback)
    rospy.spin()


if __name__ == "__main__":
     GPIO.setwarnings(False) # Ignore warning for now
     GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
     GPIO.setup(35, GPIO.OUT, initial=GPIO.LOW)
     GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW)
     GPIO.setup(37, GPIO.OUT, initial=GPIO.LOW)
     GPIO.setup(38, GPIO.OUT, initial=GPIO.LOW)
     pwm1 = GPIO.PWM(37, 100)
     pwm2 = GPIO.PWM(38, 100)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
