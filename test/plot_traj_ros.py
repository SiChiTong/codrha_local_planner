import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import os
import yaml

[os.remove(os.path.join("../log/",f)) for f in os.listdir("../log/") if f.endswith(".pdf")]
[os.remove(os.path.join("../log/",f)) for f in os.listdir("../log/") if f.endswith(".png")]

# plt.hold(True)
# plt.grid(True)
# plt.grid('on')

fig = plt.figure()
el_tree = ET.parse('../log/traj_log_CODRHAPlannerROS.xml').getroot()

with open('../log/' + 'rearanged_' + 'cmmd_log_CODRHAPlannerROS.yaml', 'r') as document:
    cmd_ts = yaml.load(document)

for segment in el_tree: # over segments
    tm = [float(el) for el in segment[0].text.split(",")]
    tmfake = [float(el) for el in segment[5].text.split(",")]

    plt.subplot(331)
    ax = fig.gca()
    plt.grid('on')

    # Control points
    # lst = segment[1].text.split(",")
    # othX = [float(el) for el in lst[0::2]]
    # othY = [float(el) for el in lst[1::2]]
    # ax.plot(othX, othY, 'o')

    # Initialization
    # lst = segment[3].text.split(",")
    # othX = [float(el) for el in lst[0::2]]
    # othY = [float(el) for el in lst[1::2]]
    # ax.plot(othX, othY, 'kx')

    # Obstacle
    for obst in segment[7]:
        lst = obst.text.split(",")
        x = float(lst[0])
        y = float(lst[1])
        innerCirc = plt.Circle((x, y), .25, color='grey', ls = 'solid', fill=True,  alpha=0.2)
        outerCirc = plt.Circle((x, y), .25+.18, color='grey', ls = 'solid', lw=0.5, fill=False, alpha=0.5)

        ax.add_artist(innerCirc)
        ax.add_artist(outerCirc)
    ax.axis('equal')

    # Some points along the trajectory
    lst = segment[2].text.split(",")
    othX = [float(el) for el in lst[0::3]]
    othY = [float(el) for el in lst[1::3]]
    othPhi = [float(el) for el in lst[2::3]]
    ax.plot(othX, othY)
    plt.title("pln path")
    # ax.plot(othX[::12], othY[::12], '|')
    # ax.axis('equal')

    plt.subplot(334)
    ax = fig.gca()
    plt.grid('on')
    ax.plot(tm, othPhi)
    plt.title("pln ori")

    plt.subplot(332)
    ax = fig.gca()
    plt.grid('on')
    lst = segment[4].text.split(",")
    vlLin = [float(el) for el in lst[0::2]]
    # vlAng = [float(el) for el in lst[1::2]]
    ax.plot(tm, vlLin)
    plt.title("pln lin v")
    ax.axis('equal')

    plt.subplot(325)
    ax = fig.gca()
    plt.grid('on')
    # print [float(el) for el in cmd_ts['twist']['linear']['x'] if el == el]
    minimum = min([float(el) for el in cmd_ts['twist']['linear']['x'] if float(el) == float(el)])
    maximum = max([float(el) for el in cmd_ts['twist']['linear']['x'] if float(el) == float(el)])
    # print maximum, minimum
    # ax.set_ylim(-0.1, 1.1)
    ax.set_xlim((-1.*cmd_ts['start_time'], max([float(el) for el in cmd_ts['stamp']])))
    ax.set_ylim((minimum-.1*abs(minimum), maximum+.1*abs(maximum)))
    ax.plot([el for el in tm], vlLin, 'r')
    ax.plot(cmd_ts['stamp'], cmd_ts['twist']['linear']['x'], 'b')

    plt.subplot(333)
    ax = fig.gca()
    plt.grid('on')
    # vlLin = [float(el) for el in lst[0::2]]
    # vlAng = [0.0] + [float(el) for el in lst[3:-1:2]] + [0.0]
    vlAng = [float(el) for el in lst[1::2]]
    ax.plot(tm, vlAng)
    plt.title("pln ang v")
    ax.axis('equal')

    plt.subplot(326)
    ax = fig.gca()
    plt.grid('on')
    # print [float(el) for el in cmd_ts['twist']['linear']['x'] if el == el]
    minimum = min([float(el) for el in cmd_ts['twist']['angular']['z'] if float(el) == float(el)])
    maximum = max([float(el) for el in cmd_ts['twist']['angular']['z'] if float(el) == float(el)])
    # print maximum, minimum
    # ax.set_ylim(-0.1, 1.1)
    ax.set_xlim((-1.*cmd_ts['start_time'],  max([float(el) for el in cmd_ts['stamp']])))
    ax.set_ylim((minimum-.1*abs(minimum), maximum+.1*abs(maximum)))
    ax.plot([el for el in tm], vlAng, 'r')
    ax.plot(cmd_ts['stamp'], cmd_ts['twist']['angular']['z'], 'b')

    plt.subplot(335)
    ax = fig.gca()
    plt.grid('on')
    lst = segment[6].text.split(",")
    vlLin = [float(el) for el in lst[0::2]]
    # vlAng = [float(el) for el in lst[1::2]]
    ax.plot(tmfake, vlLin)
    plt.title("pln lin v ext")
    ax.axis('equal')

    plt.subplot(336)
    ax = fig.gca()
    plt.grid('on')
    # vlLin = [float(el) for el in lst[0::2]]
    # vlAng = [0.0] + [float(el) for el in lst[3:-1:2]] + [0.0]
    vlAng = [float(el) for el in lst[1::2]]
    ax.plot(tmfake, vlAng)
    plt.title("pln ang v ext")
    ax.axis('equal')

fig.savefig('../log/traj_log_CODRHAPlannerROS.png', bbox_inches='tight', dpi=300)
# plt.show()
