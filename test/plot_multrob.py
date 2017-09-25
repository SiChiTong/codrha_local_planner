import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import yaml
import math
import numpy as np
from matplotlib.offsetbox import OffsetImage
from matplotlib.cbook import get_sample_data
import matplotlib.cm as cm
from scipy.interpolate import interp1d
# import cv2

def quat_to_yaw(qx, qy, qz, qw):
    # (z-axis rotation)
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)
    return yaw

file1 = 'r1.yaml'
file2 = 'r2.yaml'

# LOAD OPT TRAJ

int_x1 = []
int_y1 = []
int_t1 = []
int_time1 = []

int_x2 = []
int_y2 = []
int_t2 = []
int_time2 = []

with open(file1, 'r') as documents:
    traj = yaml.load_all(documents)

    first = True
    for segment in traj:  # over segments

        if segment == None:
            break

        if first:
            first = False
            secs = segment['header']['stamp']['secs']
            nsecs = segment['header']['stamp']['nsecs']
            stamp = secs + nsecs*1e-9
        # time_to_execute = segment['time_to_exec']
        tm = [t+stamp for t in segment['time_seq'][:-1]]
        tmfake = [t+stamp for t in segment['ext_time_seq'][:-1]]


        int_time1 = int_time1 + tm
        int_x1 = int_x1 + segment['points']['x'][:-1]
        int_y1 = int_y1 + segment['points']['y'][:-1]
        int_t1 = int_t1 + segment['points']['theta'][:-1]

with open(file2, 'r') as documents:
        traj = yaml.load_all(documents)

        first = True
        for segment in traj:  # over segments

            if segment == None:
                break

            if first:
                first = False
                secs = segment['header']['stamp']['secs']
                nsecs = segment['header']['stamp']['nsecs']
                stamp = secs + nsecs*1e-9
            # time_to_execute = segment['time_to_exec']
            tm = [t+stamp for t in segment['time_seq'][:-1]]
            tmfake = [t+stamp for t in segment['ext_time_seq'][:-1]]

            int_time2 = int_time2 + tm
            int_x2 = int_x2 + segment['points']['x'][:-1]
            int_y2 = int_y2 + segment['points']['y'][:-1]
            int_t2 = int_t2 + segment['points']['theta'][:-1]

fx1 = interp1d(np.array(int_time1), np.array(int_x1), kind='cubic')
fy1 = interp1d(np.array(int_time1), np.array(int_y1), kind='cubic')
fx2 = interp1d(np.array(int_time2), np.array(int_x2), kind='cubic')
fy2 = interp1d(np.array(int_time2), np.array(int_y2), kind='cubic')


dist = list()
init_time = max(int_time1[0], int_time2[0])
final_time = min(int_time1[-1], int_time2[-1])
# print int_time1[0]
# print int_time1[-1]
# print int_time2[0]
# print int_time2[-1]
the_time = np.linspace(init_time, final_time, num=100, endpoint=True)

for t in the_time:

    final_x1 = fx1(t)
    final_y1 = fy1(t)
    final_x2 = fx2(t)
    final_y2 = fy2(t)

    dist.append(np.sqrt((final_x1 - final_x2)**2. + (final_y1 - final_y2)**2.)-2*.2)

# PATH
fig = plt.figure()
ax = fig.gca()
plt.grid('on')
plt.hold(True)

print len(dist)
print the_time.shape
ax.plot(np.linspace(init_time, final_time, num=100, endpoint=True), dist)
# ax.axis('equal')

fig.savefig('mr.png', bbox_inches='tight', dpi=300)
fig.savefig('mr.pdf', bbox_inches='tight', dpi=300)
#
fig2 = plt.figure()
ax = fig2.gca()
plt.grid('on')
plt.hold(True)
#
ax.plot(int_x1, int_y1)
ax.plot(int_x2, int_y2)

ax.axis('equal')

fig2.savefig('mr2.png', bbox_inches='tight', dpi=300)
fig2.savefig('mr2.pdf', bbox_inches='tight', dpi=300)
