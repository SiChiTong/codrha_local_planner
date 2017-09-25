import matplotlib as mpl
mpl.use('Agg')
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

def getNColors(N, cmap):
    values = range(N)
    cm = plt.get_cmap(cmap)
    cNorm  = mpl.colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = mpl.cm.ScalarMappable(norm=cNorm, cmap=cm)
    return [scalarMap.to_rgba(val) for val in values]

plColors = getNColors(2, 'seismic')

class Robot:
    def __init__(self, myID, direc):

        self.id = myID

        with open(direc + 'tb' + str(myID) + '.yaml', 'r') as documents:
            iter_traj = yaml.load_all(documents)

            traj = list(iter_traj)

            # traj0 = traj.next()
            traj0 = traj[0]
            if traj0 != None:
                secs = traj0['header']['stamp']['secs']
                nsecs = traj0['header']['stamp']['nsecs']
                self._init_stamp = secs + nsecs*1e-9
            else:
                raise NameError('No segment in file')

            self._time = []
            self._x = []
            self._y = []
            self._theta = []
            self._radius = []

            for segment in traj:  # over segments

                if segment == None:
                    break

                print myID, segment['header']['seq']

                # time_to_execute = segment['time_to_exec']
                tm = [t + self._init_stamp for t in segment['time_seq'][:-1]]
                # tmfake = [t+stamp for t in segment['ext_time_seq'][:-1]]

                self._time.extend(tm)
                self._x.extend(segment['points']['x'][:-1])
                self._y.extend(segment['points']['y'][:-1])
                self._theta.extend(segment['points']['theta'][:-1])
                self._radius.extend([segment['radius']]*len(tm))

            # ntraj = yaml.load_all(documents)
            # seg_list = list(ntraj)
            # print len(seg_list)
            # print seg_list[-1]
            last_seg = traj[-2]
            tm = last_seg['time_seq'][-1] + self._init_stamp
            self._final_stamp = tm

            self._time.append(tm)
            self._x.append(last_seg['points']['x'][-1])
            self._y.append(last_seg['points']['y'][-1])
            self._theta.append(last_seg['points']['theta'][-1])
            self._radius.append(last_seg['radius'])

        self.x_of_t = interp1d(np.array(self._time), np.array(self._x), kind='cubic')
        self.y_of_t = interp1d(np.array(self._time), np.array(self._y), kind='cubic')
        self.theta_of_t = interp1d(np.array(self._time), np.array(self._theta), kind='cubic')
        self.rad_of_t = interp1d(np.array(self._time), np.array(self._radius), kind='cubic')

        self._time_res = .05

    def plot(self, time, fig):

        if time < self._init_stamp:
            my_time = self._init_stamp
        elif time > self._final_stamp:
            my_time = self._final_stamp
        else:
            my_time = time

        self._last_time = my_time

        ax = fig.gca()

        self.circ = plt.Circle((self.x_of_t(my_time), self.y_of_t(my_time)), self.rad_of_t(my_time), color=plColors[self.id], ls = 'solid', fill=False)

        triaVert = np.array(
            [[self.rad_of_t(my_time)*np.cos(self.theta_of_t(my_time))+self.x_of_t(my_time), \
            self.rad_of_t(my_time)*np.sin(self.theta_of_t(my_time))+self.y_of_t(my_time)],
            [self.rad_of_t(my_time)*np.cos(self.theta_of_t(my_time)-2.5*np.pi/3.0)+self.x_of_t(my_time), \
            self.rad_of_t(my_time)*np.sin(self.theta_of_t(my_time)-2.5*np.pi/3.0)+self.y_of_t(my_time)],
            [self.rad_of_t(my_time)*np.cos(self.theta_of_t(my_time)+2.5*np.pi/3.0)+self.x_of_t(my_time), \
            self.rad_of_t(my_time)*np.sin(self.theta_of_t(my_time)+2.5*np.pi/3.0)+self.y_of_t(my_time)]])

        self.tria = plt.Polygon(triaVert, color=plColors[self.id], fill=True, alpha=0.2)

        ax.add_artist(self.circ)
        ax.add_artist(self.tria)

        n_pts = int((my_time - self._init_stamp)/self._time_res)
        n_pts = max(1, n_pts)

        x = [self.x_of_t(t) for t in np.linspace(self._init_stamp, my_time, num=n_pts, endpoint=True)]
        y = [self.y_of_t(t) for t in np.linspace(self._init_stamp, my_time, num=n_pts, endpoint=True)]
        theta = [self.theta_of_t(t) for t in np.linspace(self._init_stamp, my_time, num=n_pts, endpoint=True)]

        self.path, = ax.plot(x, y, color=plColors[self.id], label='planned trajectory')

    def updatePlot(self, time):
        if (time == self._last_time):
            return

        if time < self._init_stamp:
            my_time = self._init_stamp
        elif time > self._final_stamp:
            my_time = self._final_stamp
        else:
            my_time = time

        self._last_time = my_time

        self.circ.center = self.x_of_t(my_time), self.y_of_t(my_time)

        triaVert = np.array(
            [[self.rad_of_t(my_time)*np.cos(self.theta_of_t(my_time))+self.x_of_t(my_time), \
            self.rad_of_t(my_time)*np.sin(self.theta_of_t(my_time))+self.y_of_t(my_time)],
            [self.rad_of_t(my_time)*np.cos(self.theta_of_t(my_time)-2.5*np.pi/3.0)+self.x_of_t(my_time), \
            self.rad_of_t(my_time)*np.sin(self.theta_of_t(my_time)-2.5*np.pi/3.0)+self.y_of_t(my_time)],
            [self.rad_of_t(my_time)*np.cos(self.theta_of_t(my_time)+2.5*np.pi/3.0)+self.x_of_t(my_time), \
            self.rad_of_t(my_time)*np.sin(self.theta_of_t(my_time)+2.5*np.pi/3.0)+self.y_of_t(my_time)]])

        self.tria.set_xy(triaVert)

        n_pts = int((my_time - self._init_stamp)/self._time_res)
        n_pts = max(1, n_pts)

        x = [self.x_of_t(t) for t in np.linspace(self._init_stamp, my_time, num=n_pts, endpoint=True)]
        y = [self.y_of_t(t) for t in np.linspace(self._init_stamp, my_time, num=n_pts, endpoint=True)]

        self.path.set_xdata(x)
        self.path.set_ydata(y)

        self.path.set_label('planned trajectory')

    def x(self, time):
        if time < self._init_stamp:
            my_time = self._init_stamp
        elif time > self._final_stamp:
            my_time = self._final_stamp
        else:
            my_time = time
        return self.x_of_t(my_time)
    def y(self, time):
        if time < self._init_stamp:
            my_time = self._init_stamp
        elif time > self._final_stamp:
            my_time = self._final_stamp
        else:
            my_time = time
        return self.y_of_t(my_time)
    def theta(self, time):
        if time < self._init_stamp:
            my_time = self._init_stamp
        elif time > self._final_stamp:
            my_time = self._final_stamp
        else:
            my_time = time
        return self.theta_of_t(my_time)
    def rad(self, time):
        if time < self._init_stamp:
            my_time = self._init_stamp
        elif time > self._final_stamp:
            my_time = self._final_stamp
        else:
            my_time = time
        return self.rad_of_t(my_time)

# file1 = 'r1.yaml'
# file2 = 'r2.yaml'
#
# # LOAD OPT TRAJ
#
# int_x1 = []
# int_y1 = []
# int_t1 = []
# int_time1 = []
#
# int_x2 = []
# int_y2 = []
# int_t2 = []
# int_time2 = []
#
# with open(file1, 'r') as documents:
#     traj = yaml.load_all(documents)
#
#     first = True
#     for segment in traj:  # over segments
#
#         if segment == None:
#             break
#
#         if first:
#             first = False
#             secs = segment['header']['stamp']['secs']
#             nsecs = segment['header']['stamp']['nsecs']
#             stamp = secs + nsecs*1e-9
#         # time_to_execute = segment['time_to_exec']
#         tm = [t+stamp for t in segment['time_seq'][:-1]]
#         tmfake = [t+stamp for t in segment['ext_time_seq'][:-1]]
#
#
#         int_time1 = int_time1 + tm
#         int_x1 = int_x1 + segment['points']['x'][:-1]
#         int_y1 = int_y1 + segment['points']['y'][:-1]
#         int_t1 = int_t1 + segment['points']['theta'][:-1]
#
# with open(file2, 'r') as documents:
#         traj = yaml.load_all(documents)
#
#         first = True
#         for segment in traj:  # over segments
#
#             if segment == None:
#                 break
#
#             if first:
#                 first = False
#                 secs = segment['header']['stamp']['secs']
#                 nsecs = segment['header']['stamp']['nsecs']
#                 stamp = secs + nsecs*1e-9
#             # time_to_execute = segment['time_to_exec']
#             tm = [t+stamp for t in segment['time_seq'][:-1]]
#             tmfake = [t+stamp for t in segment['ext_time_seq'][:-1]]
#
#             int_time2 = int_time2 + tm
#             int_x2 = int_x2 + segment['points']['x'][:-1]
#             int_y2 = int_y2 + segment['points']['y'][:-1]
#             int_t2 = int_t2 + segment['points']['theta'][:-1]
#
# fx1 = interp1d(np.array(int_time1), np.array(int_x1), kind='cubic')
# fy1 = interp1d(np.array(int_time1), np.array(int_y1), kind='cubic')
# fx2 = interp1d(np.array(int_time2), np.array(int_x2), kind='cubic')
# fy2 = interp1d(np.array(int_time2), np.array(int_y2), kind='cubic')


def main():

    direc = './mr/'

    # CREATE ROBOTS
    robots = [Robot(0, direc), Robot(1, direc)]

    # TIME
    init_time = min(robots[0]._time[0], robots[1]._time[0])
    final_time = max(robots[0]._time[-1], robots[1]._time[-1])

    # DISTANCE
    dist = list()

    the_time = np.linspace(init_time, final_time, num=100, endpoint=True)

    for t in the_time:

        dist.append(np.sqrt((robots[0].x(t) - robots[1].x(t))**2. + (robots[0].y(t) - robots[1].y(t))**2.)-(robots[0].rad(t) + robots[1].rad(t)))

    fig = plt.figure()
    ax = fig.gca()
    plt.grid('on')
    plt.hold(True)
    ax.plot(the_time, dist)

    # ax.axis('equal')

    fig.savefig(direc + 'mr.png', bbox_inches='tight', dpi=300)
    fig.savefig(direc + 'mr.pdf', bbox_inches='tight', dpi=300)

    print('Dist: {}'.format(min(dist)))

    # PLOT PATHS
    fig_paths = plt.figure()
    plt.grid('on')
    plt.hold(True)
    ax = fig_paths.gca()
    ax.axis('equal')

    # plot
    [r.plot(0.0, fig_paths) for r in robots]

    the_time = np.linspace(init_time, final_time, num=100, endpoint=True)

    #
    for i, t in enumerate(the_time):
        [r.updatePlot(t) for r in robots]

        ax.relim()
        ax.autoscale_view(True, True, True)
        fig_paths.canvas.draw()

        fig_paths.savefig(direc + 'mr' + str(i).zfill(3) + '.png', bbox_inches='tight', dpi=300)
        fig_paths.savefig(direc + 'mr' + str(i).zfill(3) + '.pdf', bbox_inches='tight', dpi=300)

if __name__ == "__main__":
    main()
