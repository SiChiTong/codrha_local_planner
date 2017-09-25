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
import sys
from matplotlib import transforms
# import cv2

comp_hor = .3
tf = transforms.Affine2D().rotate_deg(90)

def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█'):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    sys.stdout.write('\r{} |{}|  {}%  {}\r'.format(prefix, bar, percent, suffix))
    sys.stdout.flush()
    # Print New Line on Complete
    if iteration == total:
        print()


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

class Robot:
    def __init__(self, myID, direc, colors):

        self.id = myID
        self._colors = colors

        commun_name = 'CODRHAPlannerROS'

        print('Loading tb' + str(myID).zfill(2) + ' trajectory log...')

        with open(direc + 'traj_log_CODRHAPlannerROS' + '.yaml', 'r') as documents:
        # with open(direc + 'traj_log_map_CODRHAPlannerROS' + '.yaml', 'r') as documents:
        # with open(direc + 'traj_log_tb' + str(myID).zfill(2) + '_' + commun_name + '.yaml', 'r') as documents:
            iter_traj = yaml.load_all(documents)

            traj = list(iter_traj)

            traj0 = traj[0]
            if traj0 != None:
                secs = traj0['header']['stamp']['secs']
                nsecs = traj0['header']['stamp']['nsecs']
                self._init_stamp = secs + nsecs*1e-9
            else:
                raise NameError('No segment in file')

            self._pl_time = []
            self._x = []
            self._y = []
            self._theta = []
            self._radius = []
            self._costmap = []
            self._extended_pts_x = []
            self._extended_pts_y = []

            l = len(traj)
            printProgressBar(0, l, length = 50)
            i = 1
            for segment in traj:  # over segments

                printProgressBar(i, l, length = 50)
                # print('Loading tb' + str(myID).zfill(2) + ' log'+'.'*i, end='\r')

                if segment == None:
                    break

                if i == 1:
                    secs = segment['header']['stamp']['secs']
                    nsecs = segment['header']['stamp']['nsecs']
                    self._init_stamp = secs + nsecs*1e-9

                i += 1

                # print('tb{}::seq: {}'.format(myID, segment['header']['seq']), end='\r')

                # time_to_execute = segment['time_to_exec']
                tm = [t + self._init_stamp for t in segment['time_seq'][:-1]]
                # tmfake = [t+stamp for t in segment['ext_time_seq'][:-1]]

                self._pl_time.extend(tm)
                self._x.extend(segment['points']['x'][:-1])
                self._y.extend(segment['points']['y'][:-1])
                self._theta.extend(segment['points']['theta'][:-1])
                self._radius.extend([segment['radius']]*len(tm))
                self._extended_pts_x.append(segment['extended_points']['x'])
                self._extended_pts_y.append(segment['extended_points']['y'])

                rgb_obst = self._computeRGBCostmap(segment['costmap'], segment['radius'])

                xmdim = segment['costmap']['size_in_meters_x']
                ymdim = segment['costmap']['size_in_meters_y']
                orix = segment['costmap']['origin_x']
                oriy = segment['costmap']['origin_y']

                self._costmap.append({'time': tm[-1], 'image': rgb_obst, 'ori': (orix, oriy), 'mdim': (xmdim, ymdim)})

            # ntraj = yaml.load_all(documents)
            # seg_list = list(ntraj)
            # print len(seg_list)
            # print seg_list[-1]



            last_seg = traj[-2] # last is none
            tm = last_seg['time_seq'][-1] + self._init_stamp
            self._final_stamp = tm

            self._pl_time.append(tm)
            self._x.append(last_seg['points']['x'][-1])
            self._y.append(last_seg['points']['y'][-1])
            self._theta.append(last_seg['points']['theta'][-1])
            self._radius.append(last_seg['radius'])
            self._costmap[-1]['time'] = tm

        self.x_of_t = interp1d(np.array(self._pl_time), np.array(self._x), kind='cubic')
        self.y_of_t = interp1d(np.array(self._pl_time), np.array(self._y), kind='cubic')
        self.theta_of_t = interp1d(np.array(self._pl_time), np.array(self._theta), kind='cubic')
        self.rad_of_t = interp1d(np.array(self._pl_time), np.array(self._radius), kind='cubic')

        self._time_res = .05

        print('Loading tb' + str(myID).zfill(2) + ' feedback log...')
        approx_lenght =comp_hor/.02 * len(traj) + 1/.02

        with open(direc + 'fb_log_' + commun_name + '.yaml', 'r') as documents:
            iter_traj = yaml.load_all(documents)

            self._fb_x = []
            self._fb_y = []
            self._fb_theta = []
            self._fb_time = []
            self._fb_target_x = []
            self._fb_target_y = []
            i = 1
            for segment in iter_traj:  # over segments
                printProgressBar(i, max(approx_lenght, i+1), length = 50)

                if segment == None:
                    break

                secs = segment['header']['stamp']['secs']
                nsecs = segment['header']['stamp']['nsecs']

                self._fb_final_stamp = secs + nsecs*1e-9

                if i == 1:
                    self._fb_init_stamp = self._fb_final_stamp

                self._fb_time.append(self._fb_final_stamp)
                self._fb_x.append(segment['pose']['pose']['position']['x'])
                self._fb_y.append(segment['pose']['pose']['position']['y'])
                qx = segment['pose']['pose']['orientation']['x']
                qy = segment['pose']['pose']['orientation']['y']
                qz = segment['pose']['pose']['orientation']['z']
                qw = segment['pose']['pose']['orientation']['w']
                theta = quat_to_yaw(qx, qy, qz, qw)
                self._fb_theta.append(theta)

                self._fb_target_x.append(segment['target_pose']['pose']['position']['x'])
                self._fb_target_y.append(segment['target_pose']['pose']['position']['y'])

                i += 1

        self.fb_x_of_t = interp1d(np.array(self._fb_time), np.array(self._fb_x), kind='cubic')
        self.fb_y_of_t = interp1d(np.array(self._fb_time), np.array(self._fb_y), kind='cubic')
        self.fb_theta_of_t = interp1d(np.array(self._fb_time), np.array(self._fb_theta), kind='cubic')
        self.target_x = interp1d(np.array(self._fb_time), np.array(self._fb_target_x), kind='cubic')
        self.target_y = interp1d(np.array(self._fb_time), np.array(self._fb_target_y), kind='cubic')
        # with open(direc + 'fb_log_' + str(myID).zfill(2) + '_' + commun_name + '.yaml', 'r') as documents:

    def _computeRGBCostmap(self, costmap, rad):
        xdim = costmap['size_in_cells_x']
        ydim = costmap['size_in_cells_y']

        # obst = np.rot90(np.flipud(np.array(costmap['costmap']).reshape((ydim, xdim))))
        obst = np.rot90(np.array(costmap['costmap']).reshape((ydim, xdim)))

        r_obst = np.zeros(obst.shape)
        g_obst = np.zeros(obst.shape)
        b_obst = np.zeros(obst.shape)

        # TODO
        # Think of how to use colors[id] for changing the colors of costmap for
        # each robot

        r_obst[obst==254] = 1.
        g_obst[obst==254] = 0.
        b_obst[obst==254] = 0.

        # FIXME this info should be on them log file (or some of it)
        maxA = 253.
        cost_scaling_factor = 1.
        minA = (maxA-1.)*np.exp(-1.*cost_scaling_factor*(rad - 0.0));
        maxB = 0.9
        minB = .0

        r_obst[obst!=254] = (maxB - minB)/(minA - maxA)*obst[obst!=254] + maxB - (maxB - minB)/(minA - maxA)*minA
        g_obst[obst!=254] = (maxB - minB)/(minA - maxA)*obst[obst!=254] + maxB - (maxB - minB)/(minA - maxA)*minA
        b_obst[obst!=254] = (maxB - minB)/(minA - maxA)*obst[obst!=254] + maxB - (maxB - minB)/(minA - maxA)*minA

        r_obst[obst<minA] = 1.
        g_obst[obst<minA] = 1.
        b_obst[obst<minA] = 1.
        #
        return np.dstack((r_obst, g_obst, b_obst))

    def plot(self, time, fig):

        if time < self._init_stamp:
            pl_time = self._init_stamp
        elif time > self._final_stamp:
            pl_time = self._final_stamp
        else:
            pl_time = time

        self._last_time = pl_time

        ax = fig.gca()

        self.circ = plt.Circle((self.y_of_t(pl_time), self.x_of_t(pl_time)), self.rad_of_t(pl_time), color=self._colors[self.id], ls = 'solid', fill=False, zorder=3)
        self.circ2 = plt.Circle((self.y_of_t(pl_time), self.x_of_t(pl_time)), self.rad_of_t(pl_time), color=self._colors[self.id], ls = None, fill=True, alpha=0.2, zorder=3)

        triaVert = np.array(
            [[self.rad_of_t(pl_time)*np.cos(-1.*self.theta_of_t(pl_time)+np.pi/2.)+self.y_of_t(pl_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.theta_of_t(pl_time)+np.pi/2.)+self.x_of_t(pl_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.theta_of_t(pl_time)+np.pi/2.-2.5*np.pi/3.0)+self.y_of_t(pl_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.theta_of_t(pl_time)+np.pi/2.-2.5*np.pi/3.0)+self.x_of_t(pl_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.theta_of_t(pl_time)+np.pi/2.+2.5*np.pi/3.0)+self.y_of_t(pl_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.theta_of_t(pl_time)+np.pi/2.+2.5*np.pi/3.0)+self.x_of_t(pl_time)]])

        self.tria = plt.Polygon(triaVert, color=self._colors[self.id], fill=True, alpha=0.2, zorder=3)

        ax.add_artist(self.circ)
        ax.add_artist(self.circ2)
        ax.add_artist(self.tria)

        times = [i['time'] for i in self._costmap]
        np_tm = np.array(times)
        ind = list(np_tm < pl_time).index(False)

        self._last_costmap_index = ind

        orix = self._costmap[ind]['ori'][0]
        oriy = self._costmap[ind]['ori'][1]
        mdimx = self._costmap[ind]['mdim'][0]
        mdimy = self._costmap[ind]['mdim'][1]
        # self._costmap_img = ax.imshow(self._costmap[ind]['image'], interpolation='none', extent=[orix,  mdimx+orix, oriy, mdimy+oriy], zorder=0)
        self._costmap_img = ax.imshow(self._costmap[ind]['image'], interpolation='none', extent=[oriy,  mdimy+oriy, orix, mdimx+orix], zorder=0)

        n_pts = int((pl_time - self._init_stamp)/self._time_res)
        n_pts = max(1, n_pts)

        x = [self.x_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]
        y = [self.y_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]
        theta = [self.theta_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]

        self.path, = ax.plot(y, x, color=self._colors[self.id], label='planned')

        # print(pl_time-self._init_stamp)
        self._last_n_traj = math.ceil((pl_time-self._init_stamp)/comp_hor)
        print('')
        # print(self._last_n_traj)

        # n_pts = int((n_traj*comp_hor - self._init_stamp)/self._time_res)
        # n_pts = max(1, n_pts)

        # x = [self.x_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]
        # y = [self.y_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]
        if self._last_n_traj > 0:
            self._planned_traj, = ax.plot(self._extended_pts_y[self._last_n_traj-1], self._extended_pts_x[self._last_n_traj-1], color='blue', label='planned trajectory', linewidth=0.8)
        else:
            self._planned_traj, = ax.plot(0, 0, color='blue', linewidth=0.8)

        if time < self._fb_init_stamp:
            fb_time = self._fb_init_stamp
        elif time > self._fb_final_stamp:
            fb_time = self._fb_final_stamp
        else:
            fb_time = time

        x = [self.fb_x_of_t(t) for t in np.linspace(self._fb_init_stamp, fb_time, num=n_pts, endpoint=True)]
        y = [self.fb_y_of_t(t) for t in np.linspace(self._fb_init_stamp, fb_time, num=n_pts, endpoint=True)]

        self._fb_traj, = ax.plot(x, y, color='red', label='feedback', linewidth=0.8)

        self._fb_circ = plt.Circle((self.fb_y_of_t(fb_time), self.fb_x_of_t(fb_time)), self.rad_of_t(pl_time), color='red', ls = 'solid', fill=False, zorder=3)
        self._fb_circ2 = plt.Circle((self.fb_y_of_t(fb_time), self.fb_x_of_t(fb_time)), self.rad_of_t(pl_time), color='red', ls = None, fill=True, alpha=0.2, zorder=3)


        triaVert = np.array(
            [[self.rad_of_t(pl_time)*np.cos(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.)+self.fb_y_of_t(fb_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.)+self.fb_x_of_t(fb_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.-2.5*np.pi/3.0)+self.fb_y_of_t(fb_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.-2.5*np.pi/3.0)+self.fb_x_of_t(fb_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.+2.5*np.pi/3.0)+self.fb_y_of_t(fb_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.+2.5*np.pi/3.0)+self.fb_x_of_t(fb_time)]])

        self._fb_tria = plt.Polygon(triaVert, color='red', fill=True, alpha=0.2, zorder=3)

        ax.add_artist(self._fb_circ)
        ax.add_artist(self._fb_circ2)
        ax.add_artist(self._fb_tria)

        self._target_fb_circ, = ax.plot(self.target_y(fb_time), self.target_x(fb_time), color=self._colors[self.id], marker='o', ls='none', ms=8, mfc=self._colors[self.id], mew=2, mec=self._colors[self.id]) # red filled circle
        self._target_fb_cross, = ax.plot(self.target_y(fb_time), self.target_x(fb_time), color='k', marker='+', ls='none', ms=8, mec="w", mew=1) # white cross

    def updatePlot(self, time):
        if (time == self._last_time):
            return

        if time < self._init_stamp:
            pl_time = self._init_stamp
        elif time > self._final_stamp:
            pl_time = self._final_stamp
        else:
            pl_time = time

        self._last_time = pl_time

        self.circ.center = self.y_of_t(pl_time), self.x_of_t(pl_time)
        self.circ2.center = self.y_of_t(pl_time), self.x_of_t(pl_time)

        triaVert = np.array(
            [[self.rad_of_t(pl_time)*np.cos(-1.*self.theta_of_t(pl_time)+np.pi/2.)+self.y_of_t(pl_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.theta_of_t(pl_time)+np.pi/2.)+self.x_of_t(pl_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.theta_of_t(pl_time)+np.pi/2.-2.5*np.pi/3.0)+self.y_of_t(pl_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.theta_of_t(pl_time)+np.pi/2.-2.5*np.pi/3.0)+self.x_of_t(pl_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.theta_of_t(pl_time)+np.pi/2.+2.5*np.pi/3.0)+self.y_of_t(pl_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.theta_of_t(pl_time)+np.pi/2.+2.5*np.pi/3.0)+self.x_of_t(pl_time)]])

        self.tria.set_xy(triaVert)

        n_pts = int((pl_time - self._init_stamp)/self._time_res)
        n_pts = max(1, n_pts)

        x = [self.x_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]
        y = [self.y_of_t(t) for t in np.linspace(self._init_stamp, pl_time, num=n_pts, endpoint=True)]

        self.path.set_xdata(y)
        self.path.set_ydata(x)

        times = [i['time'] for i in self._costmap]
        np_tm = np.array(times)
        ind = list(np_tm < pl_time).index(False)

        if ind != self._last_costmap_index:

            self._last_costmap_index = ind

            orix = self._costmap[ind]['ori'][0]
            oriy = self._costmap[ind]['ori'][1]
            mdimx = self._costmap[ind]['mdim'][0]
            mdimy = self._costmap[ind]['mdim'][1]
            self._costmap_img.set_data(self._costmap[ind]['image'])
            self._costmap_img.set_extent([oriy,  mdimy+oriy, orix, mdimx+orix])
            self._costmap_img.set_zorder(0)

        self.path.set_label('planned')

        n_traj = math.ceil((pl_time-self._init_stamp)/comp_hor)
        if n_traj != self._last_n_traj:
            self._last_n_traj = n_traj

            if self._last_n_traj < len(self._extended_pts_x):

                self._planned_traj.set_ydata(self._extended_pts_x[n_traj-1])
                self._planned_traj.set_xdata(self._extended_pts_y[n_traj-1])

            else:
                self._planned_traj.set_ydata(self._extended_pts_x[-1])
                self._planned_traj.set_xdata(self._extended_pts_y[-1])

        if time < self._fb_init_stamp:
            fb_time = self._fb_init_stamp
        elif time > self._fb_final_stamp:
            fb_time = self._fb_final_stamp
        else:
            fb_time = time

        x = [self.fb_x_of_t(t) for t in np.linspace(self._fb_init_stamp, fb_time, num=n_pts, endpoint=True)]
        y = [self.fb_y_of_t(t) for t in np.linspace(self._fb_init_stamp, fb_time, num=n_pts, endpoint=True)]

        self._fb_traj.set_ydata(x)
        self._fb_traj.set_xdata(y)
        self._fb_traj.set_label('feedback')

        self._fb_circ.center = self.fb_y_of_t(fb_time), self.fb_x_of_t(fb_time)
        self._fb_circ2.center = self.fb_y_of_t(fb_time), self.fb_x_of_t(fb_time)

        triaVert = np.array(
            [[self.rad_of_t(pl_time)*np.cos(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.)+self.fb_y_of_t(fb_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.)+self.fb_x_of_t(fb_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.-2.5*np.pi/3.0)+self.fb_y_of_t(fb_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.-2.5*np.pi/3.0)+self.fb_x_of_t(fb_time)],
            [self.rad_of_t(pl_time)*np.cos(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.+2.5*np.pi/3.0)+self.fb_y_of_t(fb_time), \
            self.rad_of_t(pl_time)*np.sin(-1.*self.fb_theta_of_t(fb_time)+np.pi/2.+2.5*np.pi/3.0)+self.fb_x_of_t(fb_time)]])

        self._fb_tria.set_xy(triaVert)

        self._target_fb_circ.set_ydata(self.target_x(fb_time))
        self._target_fb_circ.set_xdata(self.target_y(fb_time))
        self._target_fb_cross.set_ydata(self.target_x(fb_time))
        self._target_fb_cross.set_xdata(self.target_y(fb_time))

    def x(self, time):
        if time < self._init_stamp:
            pl_time = self._init_stamp
        elif time > self._final_stamp:
            pl_time = self._final_stamp
        else:
            pl_time = time
        return self.x_of_t(pl_time)
    def y(self, time):
        if time < self._init_stamp:
            pl_time = self._init_stamp
        elif time > self._final_stamp:
            pl_time = self._final_stamp
        else:
            pl_time = time
        return self.y_of_t(pl_time)
    def theta(self, time):
        if time < self._init_stamp:
            pl_time = self._init_stamp
        elif time > self._final_stamp:
            pl_time = self._final_stamp
        else:
            pl_time = time
        return self.theta_of_t(pl_time)
    def rad(self, time):
        if time < self._init_stamp:
            pl_time = self._init_stamp
        elif time > self._final_stamp:
            pl_time = self._final_stamp
        else:
            pl_time = time
        return self.rad_of_t(pl_time)

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

    direc = '../log/'
    plColors = getNColors(1, 'seismic')

    # CREATE ROBOTS
    robots = [Robot(0, direc, plColors)]

    # TIME
    init_time = robots[0]._fb_time[0]
    final_time = robots[0]._fb_time[-1]

    print('\nExp time elapsed: {}'.format(final_time - init_time))
    print('Pl time elapse: {}'.format(robots[0]._pl_time[-1] - robots[0]._pl_time[0]))


    the_time = np.linspace(init_time, final_time, num=100, endpoint=True)

    if len(robots) > 1:
        # DISTANCE
        dist = list()
        for t in the_time:

            dist.append(np.sqrt((robots[0].x(t) - robots[1].x(t))**2. + (robots[0].y(t) - robots[1].y(t))**2.)-(robots[0].rad(t) + robots[1].rad(t)))

        fig = plt.figure()
        ax = fig.gca()
        plt.grid('on')
        # plt.hold(True)
        ax.plot(the_time, dist)

        # ax.axis('equal')

        fig.savefig(direc + '/demo/inter-robot-dist.png', bbox_inches='tight', dpi=300)
        fig.savefig(direc + '/demo/inter-robot-dist.pdf', bbox_inches='tight', dpi=300)

        print('Dist: {}'.format(min(dist)))

    # PLOT PATHS
    fig_paths = plt.figure()
    ax = fig_paths.gca()

    ax.invert_xaxis()

    # plt.hold(True)
    ax.grid(zorder=0)
    ax.axis('equal')


    # plot
    the_time = np.linspace(init_time, final_time, num=450, endpoint=True)

    [r.plot(the_time[0], fig_paths) for r in robots]

    print('Generating images...')

    #
    for i, t in enumerate(the_time[1:]):
        printProgressBar(i, len(the_time)-1, length = 50)

        [r.updatePlot(t) for r in robots]

        ax.relim()
        ax.set_ylim([-.3,2.])
        ax.set_xlim([-1.5,1.5])
        ax.invert_xaxis()
        # ax.autoscale_view(True, True, True)

        ax.xaxis.set_ticks(np.arange(-1.5, 1.5, .25))
        ax.yaxis.set_ticks(np.arange(.0, 2., .25))
        ax.legend()

        fig_paths.set_size_inches(6.4, 8)
        fig_paths.canvas.draw()
        fig_paths.savefig(direc + '/demo/paths' + str(i).zfill(3) + '.png', bbox_inches='tight', dpi=300)
        # fig_paths.savefig(direc + '/demo/paths' + str(i).zfill(3) + '.pdf', bbox_inches='tight', dpi=300)

    print('')

if __name__ == "__main__":
    main()
