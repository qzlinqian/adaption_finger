#!/usr/bin/env python
###############################################################################
#                           Plot paths for assembly                           #
###############################################################################

import rosbag
import matplotlib.pyplot as plt
# from matplotlib import rc
# rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
# rc('text', usetex=True)
# import numpy as np
# import quaternion
import math


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


def read_file(file, duration=0):
    bag = rosbag.Bag(file)
    topic_names1 = ['adaption/contact_force',]
    topic_names2 = ['adaption/finger_info',]
    data_set1 = {'fx': [], 'fy': [], 'f': [], 't': []}
    data_set2 = {'x': [], 'theta1': [], 'theta2': []}

    start = 0
    for topic, msg, t in bag.read_messages(topics=topic_names1):
        if start == 0 or start > t.to_sec():
            start = t.to_sec()
        if duration > 0 and t.secs - start > duration:
            break
        fx = msg.Fx
        fy = msg.Fy
        f = math.sqrt(fx * fx + fy * fy)
        data_set1['fx'].append(fx)
        data_set1['fy'].append(fy)
        data_set1['f'].append(f)
        data_set1['t'].append(msg.stamp.to_sec())
    data_set1['t'] = [x - start for x in data_set1['t']]
    start = 0
    for topic, msg, t in bag.read_messages(topics=topic_names2):
        if start == 0 or start > t.secs:
            start = t.secs
        if duration > 0 and t.secs - start > duration:
            break
        x = msg.position.x
        theta1 = msg.angle.theta1
        theta2 = msg.angle.theta2
        data_set2['x'].append(x)
        data_set2['theta1'].append(theta1)
        data_set2['theta2'].append(theta2)
    return data_set1, data_set2


def plot_data(data1, data2):
    # plt.xlim(-0.8, 0.8)
    # plt.ylim(-0.8, 0.8)
    # plt.xticks([-0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6])
    # plt.yticks([-0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8])
    # plt.axis('equal')

    plt.subplot(2,1,1)
    plt.plot(data1['t'], data1['fx'], color='c', label='Fx')
    plt.plot(data1['t'], data1['fy'], color='b', label='Fy')
    plt.plot(data1['t'], data1['f'], color='m', label='F')
    plt.legend()
    plt.xlabel('time/s')
    plt.ylabel('Force/N')

    plt.subplot(2,1,2)
    plt.plot(data2['x'], data2['theta1'], color='g', label='theta_1')
    plt.plot(data2['x'], data2['theta2'], color='b', label='theta_2')
    plt.legend()
    plt.xlabel('x/m')
    plt.ylabel('theta')


if __name__ == '__main__':
    data1, data2 = read_file(
        '../adaption_data/finger_adaption_2020-04-28-20-41-35.bag')
    plt.figure()
    plot_data(data1, data2)
    # data1 = read_file('../assemble_data/smores_assembly_2019-09-11-00-12-33.bag', 8, 440)
    # plot_data(data1)
    plt.show()
