import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET


# plt.hold(True)
# plt.grid(True)
# plt.grid('on')
cost_tweak = [1.1, 1.5, 2.0, 3.0, 4.0, 6.0, 9.0, 13.0, 16.0, 20., 25., 40.]
# for i in range(0,2):
# for i in range(7,8):
# for i in range(3,12):
for i in range(2,9):
    print i
    for j, idx in zip(cost_tweak, range(len(cost_tweak))):
        print j
# for i in range(11,12):
        fig = plt.figure()
        cost_tweak_str = str(j) if j-int(j) != 0.0 else str(int(j))
        el_tree = ET.parse('log/traj_log_v' +  str(i)  + '_' + cost_tweak_str + '_planner.xml').getroot()

        for segment in el_tree: # over segments
            tm = [float(el) for el in segment[0].text.split(",")]

            plt.subplot(221)
            ax = fig.gca()
            plt.grid('on')

            # Control points
            # lst = segment[1].text.split(",")
            # othX = [float(el) for el in lst[0::2]]
            # othY = [float(el) for el in lst[1::2]]
            # ax.plot(othX, othY, 'o')

            # Initialization
            lst = segment[3].text.split(",")
            othX = [float(el) for el in lst[0::2]]
            othY = [float(el) for el in lst[1::2]]
            # ax.plot(othX, othY, 'kx')

            # Obstacle
            for obst in segment[7]:
                lst = obst.text.split(",")
                x = float(lst[0])
                y = float(lst[1])
                innerCirc = plt.Circle((x, y), .8, color='grey', ls = 'solid', fill=True,  alpha=0.2)
                outerCirc = plt.Circle((x, y), .8+.5, color='grey', ls = 'solid', lw=0.5, fill=False, alpha=0.5)

                ax.add_artist(innerCirc)
                ax.add_artist(outerCirc)
            ax.axis('equal')

            # Some points along the trajectory
            lst = segment[2].text.split(",")
            othX = [float(el) for el in lst[0::3]]
            othY = [float(el) for el in lst[1::3]]
            othPhi = [float(el) for el in lst[2::3]]
            ax.plot(othX, othY)
            # ax.plot(othX[::12], othY[::12], '|')
            # ax.axis('equal')

            plt.subplot(222)
            ax = fig.gca()
            plt.grid('on')
            ax.plot(tm, othPhi)

            plt.subplot(223)
            ax = fig.gca()
            plt.grid('on')
            lst = segment[4].text.split(",")
            vlLin = [float(el) for el in lst[0::2]]
            vlAng = [float(el) for el in lst[1::2]]
            ax.plot(tm, vlLin)
            ax.axis('equal')

            tmfake = [float(el) for el in segment[5].text.split(",")]

            plt.subplot(224)
            ax = fig.gca()
            plt.grid('on')
            lst = segment[6].text.split(",")
            vlLin = [float(el) for el in lst[0::2]]
            vlAng = [float(el) for el in lst[1::2]]
            ax.plot(tmfake, vlLin)
            ax.axis('equal')

        fig.savefig('log/traj_log_v' +  str(i)  + '_' + cost_tweak_str + '_planner.png', bbox_inches='tight', dpi=300)
        fig.clf()
# plt.show()
