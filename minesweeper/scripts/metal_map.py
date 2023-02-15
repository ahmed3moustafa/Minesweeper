# import matplotlib.pyplot as plt
# from  matplotlib.animation import FuncAnimation
# from matplotlib import colors, style
# import rospy
# import numpy as np
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist, Pose2D
# import string
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image


# class my_metal_map:
#     def __init__(self):

#         self.n = 20
#         self.data = np.random.rand(self.n, self.n) * np.nan
#         cmap = colors.ListedColormap(['red', 'blue','green'])

#         self.fig,self.ax = plt.subplots()
#         self.im = self.ax.imshow(self.data, cmap=cmap ,extent=[0, self.n, 0, self.n], zorder=2)

#         # draw gridlines
#         self.ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)


#         alphabets = string.ascii_uppercase[:self.n-1]
#         ticklabels = [0] + list(alphabets)
#         self.ax.set_xticks(np.arange(0, self.n, 1), minor=False)
#         self.ax.set_xticklabels(ticklabels, minor=False)
#         # self.ax.invert_xaxis()

#         self.ax.set_yticks(np.arange(0, self.n, 1), minor=False);
#         self.ax.invert_yaxis()
#         self.ax.set_yticklabels(np.arange(20, 0, -1), minor=False)
#         # print(self.data)
#         # plt.show()

#     def node_init(self):
#         rospy.init_node('map', anonymous=True)
#         rospy.Subscriber("/search_complete", Bool, self.search_complete_callback)
#         rospy.Subscriber("/mine_found", Pose2D, self.map_callback)
#         # bridge = CvBridge()
#         # image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
#         # rospy.init_node("image_publisher", anonymous=True)
#         rate = rospy.Rate(10) 
#         rospy.spin()


#     def map_callback(self, mines):
#         rospy.loginfo("Received map")

#         x= int(np.floor(mines.x))
#         y= int(np.floor(mines.y))
#         self.data [x, y] = 1
#         print("done")
#         self.ani = FuncAnimation(self.fig, self.map_callback, frames=100, interval=2000)
#         print(self.data)
#         plt.ion()
#         plt.show()

#         # self.plot_map()
        
#     def search_complete_callback(self, search_complete):
#         rospy.loginfo("Received search complete")

#         # self.plot_map()

#     # def plot_map(self):
#         # set some values to 0, 1, 2



# if __name__ == '__main__':
#     my_map = my_metal_map()
#     my_map.node_init()
  

import matplotlib.pyplot as plt
from  matplotlib.animation import FuncAnimation
from matplotlib import colors, style
import rospy
import math
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose2D
import string
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class my_metal_map:
    def __init__(self):
        
        self.n = 20
        self.data_loc = np.array([[0,0]])
        self.data_loc2 = np.array([[0,0]])
        self.unique_surface_mines = np.array([[0,0]])
        self.unique_buried_mines = np.array([[0,0]])
        self.vector_output=np.array([[0,0]])
        self.vector_buried_output=np.array([[0,0]])
        self.x = 0
        self.y = 0
        self.z = 0

    def node_init(self):
        rospy.init_node('map', anonymous=True)
        rospy.Subscriber("/mine_found", Pose2D, self.map_callback)
        rospy.Subscriber("/buried_mine", Pose2D, self.map_callback_buried)
        rate = rospy.Rate(10) 
        rospy.spin()


    

    def map_callback_buried(self, mines):
        rospy.loginfo("Received map_buried")
        x= int(np.floor(mines.x))
        y= int(np.floor(mines.y))
        self.data_loc2=np.append(self.data_loc2, [[x,y]], axis=0)
        self.unique_buried_mines=np.unique(self.data_loc2, axis=0)
        for i in range(len(self.unique_buried_mines)):
            if i == 0:
                alphabets_x = 0
                self.vector_buried_output=np.append(self.vector_buried_output,[[alphabets_x,self.unique_buried_mines[i][1]]],axis=0)

            else:
                alphabets_x = string.ascii_uppercase[self.unique_buried_mines[i][0]+1]
                self.vector_buried_output=np.append(self.vector_buried_output,[[alphabets_x,self.unique_buried_mines[i][1]]],axis=0)
            self.vector_buried_output=np.unique(self.vector_buried_output, axis=0)
        print(x,y)
        print("vector Representation:\n", self.vector_buried_output)
        print("done")
        print(self.unique_buried_mines)

    def map_callback(self, mines):
        # self.data_loc = np.array([[0,0]])
        rospy.loginfo("Received map")
        self.x= int(np.floor(mines.x))
        self.y= int(np.floor(mines.y))
        self.z= int(np.floor(mines.theta))

        self.data_loc=np.append(self.data_loc, [[self.x,self.y]], axis=0)
        self.unique_surface_mines=np.unique(self.data_loc, axis=0)
        for i in range(len(self.unique_surface_mines)):
            if self.unique_surface_mines[i][0] == 0:
                alphabets_x = 0
                self.vector_output=np.append(self.vector_output,[[alphabets_x,self.unique_surface_mines[i][1]]],axis=0)
            else:
                alphabets_x = string.ascii_uppercase[self.unique_surface_mines[i][0]]
                self.vector_output=np.append(self.vector_output,[[alphabets_x,self.unique_surface_mines[i][1]]],axis=0)
            self.vector_output = np.unique(self.vector_output,axis=0) 
        print("vector Representation surface:\n", self.vector_output)
        np.savetxt('/home/anas/surface_mines.csv', self.vector_output, fmt='%s', delimiter=',')
        np.savetxt('/home/anas/surface_mines_coordinates.csv', self.unique_surface_mines, fmt='%s', delimiter=',')

        print(self.x,self.y)
        print("done")
        print(self.unique_surface_mines)



if __name__ == '__main__':
    my_map = my_metal_map()
    my_map.node_init()
  

