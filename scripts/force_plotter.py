import sys
from os import listdir
import math
import rosbag
import matplotlib.pyplot as plt

if __name__ == "__main__":
    dir = "/home/sensor/rosbags/"
    
    dir_list = listdir(dir)
    
    for path in dir_list:
        if path[-4:] == ".bag":
            fig, axs = plt.subplots(2, 1)
            axs[0].set_xlim(10,32)
            axs[1].set_xlim(10,32)
            axs[1].set_ylim(0,0.02)
            
            test_name = path.split('.')[0]
                    
            bag = rosbag.Bag(dir + path)
                        
            topics = ['/transformed_world']
            
            x, y = [], []
            
            first_loop = True
            for topic, msg, t in bag.read_messages(topics=topics):
                if first_loop:
                    start = t.to_sec()
                    first_loop = False
                x.append(t.to_sec()-start)
                y.append(math.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2))
                
            axs[0].plot(x, y, label='bar')
            
            topics = ['/leader/wrench']
            
            x, y = [], []
            first_loop = True
            for topic, msg, t in bag.read_messages(topics=topics):
                if first_loop:
                    start = t.to_sec()
                    first_loop = False
                x.append(t.to_sec()-start)
                y.append(math.sqrt(msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.force.z**2))
                
            axs[0].plot(x, y, label='follower')
            
            topics = ['/follower/wrench']
            
            x, y = [], []
            first_loop = True
            for topic, msg, t in bag.read_messages(topics=topics):
                if first_loop:
                    start = t.to_sec()
                    first_loop = False
                x.append(t.to_sec()-start)
                y.append(math.sqrt(msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.force.z**2))
                
            axs[0].plot(x, y, label='leader')    
            
            
            topics = ['/gt_error']
            
            x, y = [], []
            first_loop = True
            for topic, msg, t in bag.read_messages(topics=topics):
                if first_loop:
                    start = t.to_sec()
                    first_loop = False
                x.append(t.to_sec()-start)
                y.append(msg.magnitude)
            
            axs[1].plot(x, y, label='error')    
            axs[0].legend(loc="upper left")
            
            plt.show()
