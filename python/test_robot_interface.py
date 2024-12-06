#!/usr/bin/python3

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

import time

import rospkg

'''
Load urdf and srdf
'''
g1_description_folder = rospkg.RosPack().get_path('g1_description')
urdf_path = g1_description_folder + "/g1_29dof.urdf"
srdf_path = g1_description_folder + "/g1_29dof.srdf"

urdf = open(urdf_path, 'r').read()
srdf = open(srdf_path, 'r').read()

'''
Build ModelInterface and RobotStatePublisher
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'Unitree')
cfg.set_bool_parameter('is_model_floating_base', True)

robot = xbot.RobotInterface(cfg)

qhome = robot.getRobotState('home')
qinit = robot.getPositionReference()

alpha = 0.01
q = qinit
while True:
    q = alpha * qhome + (1 - alpha) * q
    robot.setPositionReference(q)
    robot.move()
    time.sleep(0.01)

exit()