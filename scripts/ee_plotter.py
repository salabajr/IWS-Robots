#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import matplotlib.pyplot as plt
import matplotlib.patches as patches

if __name__ == "__main__":
    rospy.init_node('ee_plotter')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    fig.canvas.draw()
    ax.set(xlabel='Y (m)', ylabel='Z (m)', title='Y-Z Plane')
    ax.set_xlim([0.2, 0.8])
    ax.set_ylim([0, 0.6])
    
    ax.grid()
    fig.canvas.draw()
    
    rate = rospy.Rate(60)
    
    leader_trail, follower_trail = {'x':[],'y':[]}, {'x':[],'y':[]}
    leader_pt, follower_pt = None, None
    
    d_list = []
    while not rospy.is_shutdown():
        loop_start_time = rospy.get_time()
        
        try:
            leader_trans = tfBuffer.lookup_transform('world', 'leader_tcp', rospy.Time())
            follower_trans = tfBuffer.lookup_transform('world', 'follower_tcp', rospy.Time())
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        leader_x = leader_trans.transform.translation.y
        leader_y = leader_trans.transform.translation.z
        follower_x = follower_trans.transform.translation.y
        follower_y = follower_trans.transform.translation.z
        
        leader_trail['x'].append(leader_x)
        leader_trail['y'].append(leader_y)
        follower_trail['x'].append(follower_x)
        follower_trail['y'].append(follower_y)
        
        if len(leader_trail['x']) > 20:
            leader_trail['x'].pop(0)
            leader_trail['y'].pop(0)
            follower_trail['x'].pop(0)
            follower_trail['y'].pop(0)
        
        if leader_pt is not None:
            leader_pt.remove()
            follower_pt.remove()
            l_trail.pop(0).remove()
            f_trail.pop(0).remove()
            dist_text.remove()
            avg_text.remove()
        
        d = math.sqrt((leader_x-follower_x)**2 + (leader_y-follower_y)**2)*1000
        
        d_list.append((rospy.get_time(), d))
        
        start = 0
        for i in range(len(d_list)):
            if d_list[i][0] > loop_start_time - 2:
                start = i
                break
        
        d_list = d_list[start:]
        
        d_sum = 0
        for t, dist in d_list:
            d_sum += dist
        
        avg = d_sum/len(d_list)
        
        props = dict(boxstyle='round', facecolor='lightgray', alpha=1)
        dist_text = ax.text(0.51, 0.51, "Instantaneous Distance: {:.2f} mm".format(d), fontsize=10, weight='bold', bbox=props)
        
        props = dict(boxstyle='round', facecolor='lightgray', alpha=1)
        avg_text = ax.text(0.51, 0.48, "Moving Average (2s): {:.2f} mm".format(avg), fontsize=10, weight='bold', bbox=props)
            
        l_trail = ax.plot(leader_trail['x'], leader_trail['y'], c='b', alpha=0.5)
        f_trail = ax.plot(follower_trail['x'], follower_trail['y'], c='r', alpha=0.5)
        
        leader_pt = ax.scatter(x=leader_x, y=leader_y, s=30, c='b', label='leader_tcp')
        follower_pt = ax.scatter(x=follower_x, y=follower_y, s=30, c='r', label='follower_tcp')
        
        ax.legend()
        fig.canvas.draw()
        