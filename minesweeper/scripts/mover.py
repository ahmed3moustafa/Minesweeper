import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

def callback(data):
    if data.linear.x > 0:
        GPIO.output(35, GPIO.HIGH)
        GPIO.output(36, GPIO.HIGH)
        GPIO.output(37, GPIO.LOW)
        GPIO.output(38, GPIO.LOW)
    elif data.linear.x < 0:
        GPIO.output(35, GPIO.LOW)
        GPIO.output(36, GPIO.LOW)
        GPIO.output(37, GPIO.HIGH)
        GPIO.output(38, GPIO.HIGH)
    else:
        GPIO.output(35, GPIO.LOW)
        GPIO.output(36, GPIO.LOW)
        GPIO.output(37, GPIO.LOW)
        GPIO.output(38, GPIO.LOW)
    if data.angular.z > 0:
        GPIO.output(35, GPIO.LOW)
        GPIO.output(36, GPIO.HIGH)
        GPIO.output(37, GPIO.HIGH)
        GPIO.output(38, GPIO.LOW)
    elif data.angular.z < 0:
        GPIO.output(35, GPIO.HIGH)
        GPIO.output(36, GPIO.LOW)
        GPIO.output(37, GPIO.LOW)
        GPIO.output(38, GPIO.HIGH)
    else:
        GPIO.output(35, GPIO.LOW)
        GPIO.output(36, GPIO.LOW)
        GPIO.output(37, GPIO.LOW)
        GPIO.output(38, GPIO.LOW)



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
    try:
        listener()
    except rospy.ROSInterruptException:
        pass