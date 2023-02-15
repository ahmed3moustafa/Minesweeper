import rospy
import time
import numpy as np
import tf.transformations 
from std_msgs.msg import Float32MultiArray, Bool, Int16
from nav_msgs.msg import Odometry
from sensor_msgs.msg import  Imu
from geometry_msgs.msg import Twist, Pose2D


class minesweeper_bot:
    def __init__ (self):
        self.minsweeper_goal = Pose2D()
        self.minsweeper_velocity = Twist()
        self.minsweeper_current = Pose2D()
        
        self.sigma_yaw=0.25
        self.sigma_yaw_dot=0.3
        self.dt=0.001 #make accurate time
        self.A = np.array([[1, 0,0],[0, 1,0],[00,0,1]])
        self.Q = np.diag([self.sigma_yaw**2, self.sigma_yaw_dot**2,self.sigma_yaw_dot**2]) ###
        self.R = np.diag([self.sigma_yaw_dot**2, self.sigma_yaw_dot**2,self.sigma_yaw_dot**2])####
        self.C = np.array([[0,0,0.5*self.dt**2],[0,0,0],[0,0,1]])
        self.I = np.eye(3)
        self.x0 = np.array([[0,0,0]])
        self.P0 = np.diag([200,200,200])

        self.kp = 0.3
        self.ki = 0.1
        self.kd =0.0
        self.kp_alpha = 0.8
        self.beta = 0.1

        i = 1
        z = 20
        x=[]
        y = []
        while z!=0:
            for j in np.arange(1,20):
                y.append(j)
                x.append (i)
            z-=1
            i += 1
            for j in np.arange(19,0,-1):
                y.append(j)
                x.append (i)
            z-=1
            i+=1

        self.goal_array=np.array([x,y]).T


        # self.goal_array = np.array([[-1,1],[1,1],[1.5,1.5],[-1.5,1.5],[-1.5,1],[1.5,1],[1.5,0],
        # [-1.5,0],[-1.5,1],[1.5,-1],[1.5,-1.5],[-1.5,-1.5],
        # [-1.5,-2],[1.5,-2],[1.5,-2.5],[-1.5,-2.5],[-1.5,-3]])   
        self.i = 0
        self.flag = False

    def node_init(self):
        rospy.init_node("minesweeper_bot",anonymous=True)
        # rospy.Subscriber("/pioneer/goal", Pose2D, self.goal_recieved_callback)    ##    
        rospy.Subscriber("/odom",Odometry,  self.odometry_callback)    
        rospy.Subscriber("/imu",Imu,   self.imu_callback)

        # rospy.Subscriber("/noisy_state",Float32MultiArray,   self.noisy_state_callback) #uncomment for kalman filter
        
        #metaldetector
        rospy.Subscriber("/metal_detector",Int16, self.metal_detector_callback)
        self.mine_found_pub=rospy.Publisher('/mine_found', Pose2D,queue_size=10)
        rospy.spin()
       

    def imu_callback(self,imu_data_msg):
        self.imu_quat = [
            imu_data_msg.orientation.x,
            imu_data_msg.orientation.y,
            imu_data_msg.orientation.z,
            imu_data_msg.orientation.w,
        ]
        self.roll_imu,  self.pitch_imu,  self.yaw_imu = tf.transformations.euler_from_quaternion( self.imu_quat)
        self.yaw_dot = imu_data_msg.angular_velocity.z
        self.linear_x_imu = imu_data_msg.linear_acceleration.x 
        self.imu_measurement = np.array([[self.linear_x_imu],[0],[self.yaw_imu]])
    
    def odometry_callback(self,odom_data):
        
        self.minsweeper_pose_x_n_noise = odom_data.pose.pose.position.x
        self.minsweeper_pose_y_n_noise = odom_data.pose.pose.position.y
        

        #uncomment for kalman filter
        # self.minsweeper_pose_x= np.random.normal(self.minsweeper_pose_x_n_noise,0.25,1)[0]
        # self.minsweeper_pose_y= np.random.normal(self.minsweeper_pose_y_n_noise,0.25,1)[0]
       
        #comment for kalman filter
        self.minsweeper_pose_x= self.minsweeper_pose_x_n_noise
        self.minsweeper_pose_y= self.minsweeper_pose_y_n_noise

        explicit_quat = [odom_data.pose.pose.orientation.x,
        odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z,
        odom_data.pose.pose.orientation.w]
        roll, pitch, self.yaw = tf.transformations.euler_from_quaternion(explicit_quat)

        #uncomment for kalman filter
        # self.minsweeper_pose_theta= np.random.normal(self.yaw,0.25,1)[0]

        #comment for kalman filter
        self.minsweeper_pose_theta= self.yaw

        self.minsweeper_current.x =self.minsweeper_pose_x
        self.minsweeper_current.y = self.minsweeper_pose_y
        self.minsweeper_current.theta = self.minsweeper_pose_theta
        ## speed
        
        
        # rospy.loginfo("pose x: %f", self.minsweeper_pose_x, "pose y: %f", self.minsweeper_pose_y, "pose theta: %f", self.minsweeper_pose_theta)

    def metal_detector_callback(self,metal_detector_data_msg):
        self.metal_detector_data = metal_detector_data_msg.data
        if self.metal_detector_data == 1:
            self.minsweeper_current.theta = 1
            print("mine_found")
        if self.metal_detector_data == 2:
            self.minsweeper_current.theta = 2
            print("burried_mine_found")
        self.mine_found_pub.publish(self.minsweeper_current)

 

    def goal_recieved_callback(self,goal_data_msg):
      
        for self.i in range(len(self.goal_array)):
            self.prev_err = 0
            self.acc_err = 0
            self.x_g = self.goal_array[self.i][0]
            self.y_g = self.goal_array[self.i][1]
            print("goal x:", self.x_g, "goal y:", self.y_g)
            theta_g = 0
            self.rho = np.sqrt(np.power(self.x_g - self.minsweeper_pose_x,2) + np.power(self.y_g - self.minsweeper_pose_y,2)) 
            turtle_speed = Twist()
            print(self.i)
            loop_time = time.time()
            while not self.rho<0.05:
                start = time.time()
                self.rho = (np.sqrt(np.power(self.x_g - self.minsweeper_pose_x,2) + np.power(self.y_g - self.minsweeper_pose_y,2))) - 1
                alpha = np.arctan2((self.y_g - self.minsweeper_pose_y),(self.x_g - self.minsweeper_pose_x)) -self.minsweeper_pose_theta
                # alpha = np.mod( alpha+np.pi, 2*np.pi) - np.pi
                # print("alpha",alpha)
                end = time.time()
                delta_t = end - start
                self.acc_err = self.acc_err*delta_t + self.acc_err
                self.prev_err = (self.rho-self.prev_err)*delta_t
                turtle_speed.linear.x = self.kp*self.rho +  self.acc_err*self.ki + self.prev_err * self.kd 
                turtle_speed.angular.z = (self.kp_alpha * alpha)
                # self.velocity_pub.publish(turtle_speed)
                # self.current_pose.publish(self.minsweeper_current)
                rospy.sleep(0.01)
            print("Reach position")
            print("position x: ", self.minsweeper_pose_x, "position y: ", self.minsweeper_pose_y, "position theta: ", self.minsweeper_pose_theta)
            end_loop = time.time()
            print("total time = ", end_loop-loop_time) 

            rospy.sleep(2) #delay for 2 seconds to detect the mines

            if self.i == len(self.goal_array)-1:
                turtle_speed.linear.x = 0
                turtle_speed.angular.z = 0
                self.velocity_pub.publish(turtle_speed)
            rospy.sleep(0.5)


#uncomment for kalman filter
    # def noisy_state_callback(self, state_msg):
    #     print("sui")
    #     self.x_hat = np.array([[self.minsweeper_current.x,self.minsweeper_current.y, self.minsweeper_current.theta]]).T
    #     print("x_hat 111",self.x_hat)
    #     self.p = (self.A.dot(self.P0).dot(self.A.T)) + self.Q
    #     self.k = self.p.dot(self.C.T).dot(np.linalg.inv(self.C.dot(self.p).dot(self.C.T)+self.R))
    #     self.x_hat = self.x_hat + self.k.dot((self.imu_measurement-self.C.dot(self.x_hat)))
    #     print("x_hat",self.x_hat)
    #     self.filtered_heading = []
    #     self.filtered_heading.append(float(self.x_hat[0]))
    #     self.filtered_heading.append(float(self.x_hat[1]))
    #     self.filtered_heading.append(float(self.x_hat[2]))
    #     self.filtered_heading_pub.publish(Float32MultiArray(data = self.filtered_heading))
    #     print(self.filtered_heading)
    #     self.x0 = self.x_hat
    #     self.P0 = self.p

        

    
 

if __name__=='__main__':
    minesweeper = minesweeper_bot()
    minesweeper.node_init()






