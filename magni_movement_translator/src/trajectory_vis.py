import re
import matplotlib.pyplot as plt
from collections import OrderedDict

def load_file(filename):
    time_array = []
    pose_array = []
    
    pose_dict =dict()
    fp = open(filename, 'r')

    for line in fp:
        time =re.findall("<time>(.*?)</time>",line)
        pose =re.findall("<pose>(.*?)</pose>",line)

        if time:
            time_array.append(time)

        if pose:
            pose_array.append(pose)
    
    if(len(time_array)!=len(pose_array)):
        print("Erorrrr")
        return

    for (time_i,pose_i) in zip(time_array,pose_array):
        pose_dict[tuple(time_i)]=tuple(pose_i)
        
    dictionary_items = pose_dict.items()
    sorted_items = sorted(dictionary_items)
    
    return sorted_items

def plot_xy(time_pose_dict):
    time_array = []
    x_array = []
    y_array = []
    z_array = []
    roll_array =[]
    pitch_array = []
    yaw_array = []
    for key in time_pose_dict:
        time = float(key[0][0])
        if(time!=0):
            time +=0.5
        time_array.append(time)

        x_array.append(float(key[1][0].split(" ")[0])*3+2)
        y_array.append(float(key[1][0].split(" ")[1])*3+2)
        z_array.append(key[1][0].split(" ")[2])
        roll_array.append(key[1][0].split(" ")[3])
        pitch_array.append(key[1][0].split(" ")[4])
        yaw_array.append(key[1][0].split(" ")[5])

    # plt.plot(x_array,y_array)
    # plt.show()

    file_pointer = open("new_trajectory.txt", "w")

    for (t,x,y,z,row,pitch,yaw) in zip(time_array,x_array,y_array,z_array,roll_array,pitch_array,yaw_array):
        file_pointer.write("<waypoint>\n")
        file_pointer.write("<time>"+str(t)+"</time>\n")
        file_pointer.write("<pose>")
        file_pointer.write(str(x)+" "+str(y)+" "+str(z)+" "+str(row)+" "+str(pitch)+" "+str(yaw))
        file_pointer.write("</pose>\n")
        file_pointer.write("</waypoint>\n")

    file_pointer.close()
    
time_pose_dict = load_file("trajectory_vis.txt")
plot_xy(time_pose_dict)