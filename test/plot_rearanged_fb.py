#!/usr/bin/env python
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
import math
import numpy as np

# INITIALIZATION


def quat_to_yaw(qx, qy, qz, qw):
    # (z-axis rotation)
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)
    return yaw


prefix = '../log/'
filename = 'fb_log_CODRHAPlannerROS'

with open(prefix + 'rearanged_' + filename + '.yaml', 'r') as document:
    odom_ts = yaml.load(document)

yaw = [
    quat_to_yaw(x, y, z, w)
    for x, y, z, w in zip(odom_ts['pose']['orientation']['x'], odom_ts['pose'][
        'orientation']['y'], odom_ts['pose']['orientation']['z'], odom_ts[
            'pose']['orientation']['w'])
]

# PLOT

# dx, dy =
# figsize = plt.figaspect(float(dy * nrows) / float(dx * ncols))
figsize = plt.figaspect(1)

fig = plt.figure(figsize=figsize)
# fig, axes = plt.subplots(2, 2, figsize=figsize)
fig.subplots_adjust(bottom=0.2)
fig.subplots_adjust(hspace=0.6)

# Position axes
plt.subplot(221)
ax = fig.gca()
plt.grid('on')
ax.plot(odom_ts['pose']['position']['x'], odom_ts['pose']['position']['y'], linewidth=0.3)
plt.axis('equal')

footprint = .2
previous_t = odom_ts['stamp'][0]
for idx, t in enumerate(odom_ts['stamp'][1:]):
    if int(t) != int(previous_t):
        # plot a little robot
        triaVert = np.array(
            [[footprint*np.cos(yaw[idx+1])+odom_ts['pose']['position']['x'][idx+1], \
            footprint*np.sin(yaw[idx+1])+odom_ts['pose']['position']['y'][idx+1]],
            [footprint*np.cos(yaw[idx+1]-2.5*np.pi/3.0)+odom_ts['pose']['position']['x'][idx+1], \
            footprint*np.sin(yaw[idx+1]-2.5*np.pi/3.0)+odom_ts['pose']['position']['y'][idx+1]],
            [footprint*np.cos(yaw[idx+1]+2.5*np.pi/3.0)+odom_ts['pose']['position']['x'][idx+1], \
            footprint*np.sin(yaw[idx+1]+2.5*np.pi/3.0)+odom_ts['pose']['position']['y'][idx+1]]])

        triang = plt.Polygon(
            triaVert, color=[.8, 0, 0], fill=True, alpha=0.2)

        # ax.add_artist(circ)
        ax.add_artist(triang)
        previous_t = t

plt.xlabel('x')
plt.ylabel('y')
plt.title('xy path', fontsize=10)
# ax.set_aspect('equal', 'datalim')
# ax.set(aspect=1)
# plt.axes().set_aspect('equal', 'datalim')

# orientation axes
plt.subplot(222)
ax = fig.gca()
plt.grid('on')
ax.plot(odom_ts['stamp'], yaw, linewidth=0.5)
plt.xlabel('yaw')
plt.ylabel('time')
plt.title('yaw', fontsize=10)
# ax.set_aspect('equal', 'datalim')
# plt.axes().set_aspect('equal', 'datalim')

# Linear velocity axes
plt.subplot(223)
ax = fig.gca()
plt.grid('on')
ax.plot(odom_ts['stamp'], odom_ts['twist']['linear']['x'], linewidth=0.5)
plt.xlabel('u')
plt.ylabel('time')
plt.title('linear velocity', fontsize=10)
# ax.set_aspect('equal', 'datalim')
# plt.axes().set_aspect('equal', 'datalim')

# Angular velocity axes
plt.subplot(224)
ax = fig.gca()
plt.grid('on')
ax.plot(odom_ts['stamp'], odom_ts['twist']['angular']['z'], linewidth=0.5)
plt.xlabel(r'$\omega$')
plt.ylabel('time')
plt.title('angular velocity', fontsize=10)
# ax.set_aspect('equal', 'datalim')
# plt.axes().set_aspect('equal', 'datalim')

fig.savefig(prefix + 'turtlebot_fb.png', bbox_inches='tight', dpi=300)
fig.savefig(prefix + 'turtlebot_fb.pdf', bbox_inches='tight', dpi=300)

# plt.show()
