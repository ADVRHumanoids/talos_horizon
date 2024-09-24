#!/usr/bin/python3

from cartesian_interface.pyci_all import *
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, analyzer, utils
from horizon.transcriptions import integrators
from horizon.ros import replay_trajectory
from horizon.utils import plotter
import matplotlib.pyplot as plt
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline

from sensor_msgs.msg import Joy
from cogimon_controller.msg import WBTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PointStamped

from matplotlib import pyplot as hplt

from matlogger2 import matlogger

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os

robot = None
def play_on_robot(solution, model:FullModelInverseDynamics):
    if robot is None:
        raise print('RobotInterface not available')

    q = np.empty(solution['q'].shape)
    qdot = np.empty(solution['v'].shape)
    qddot = np.empty(solution['a'].shape)
    tau = np.empty(solution['a'].shape)

    for i in range(solution['a'].shape[1]):
        q[:, i] = solution['q'][:, i]
        qdot[:, i] = solution['v'][:, i]
        qddot[:, i] = solution['a'][:, i]

        force_map = dict()
        for frame in model.fmap:
            force_map.update({frame: solution[f'f_{frame}'][:, i]})

        tau[:, i] = np.array(model.id_fn.call(q[:, i], qdot[:, i], qddot[:, i], force_map)).flatten()

    # execute
    for i in range(q.shape[1]):
        robot.setPositionReference(q[7:, i])
        robot.setVelocityReference(qdot[6:, i])
        robot.setEffortReference(tau[6:, i])
        robot.move()
        rospy.sleep(dt)


rospy.init_node('cogimon_straight_node')

'''
Load urdf and srdf
'''
cogimon_urdf_folder = rospkg.RosPack().get_path('cogimon_urdf')
cogimon_srdf_folder = rospkg.RosPack().get_path('cogimon_srdf')

urdf = open(cogimon_urdf_folder + '/urdf/cogimon.urdf', 'r').read()
srdf = open(cogimon_srdf_folder + '/srdf/cogimon.srdf', 'r').read()

tmp = srdf.split('<disable_collisions')
srdf = tmp[0] + '</robot>'

'''
Build ModelInterface
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

model_xbot = xbot.ModelInterface(cfg)
qhome = model_xbot.getRobotState("home")
model_xbot.setJointPosition(qhome)
model_xbot.update()

try:
    robot = xbot.RobotInterface(cfg)
    print('RobotInterface created!')
except:
    print('RobotInterface not created')

if robot is not None:
    q = robot.getPositionReference()
    q_init = robot.eigenToMap(q)
else:
    q_init = model_xbot.getJointPositionMap()

base_pose = np.array([0.03, 0., 0.962, 0., -0.029995, 0.0, 0.99955])
base_twist = np.zeros(6)

'''
Initialize Horizon problem
'''
scale = 3
ns = 120 * scale
T = 5
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose)

white_list_indices = list()
# white_list = ['LHipSag', 'LHipLat']
# white_list = ['LAnklePitch', 'RAnklePitch']
white_list = ['LHipLat', 'RHipLat', 'LAnklePitch', 'RAnklePitch']
for joint in white_list:
    white_list_indices.append(model.joint_names.index(joint))
prb.createResidual('postural_white_list', 1. * model.q[white_list_indices])

qmin = np.array(model.kd.q_min())
qmax = np.array(model.kd.q_max())
prb.createResidual("q_limit_lower", 100 * utils.barrier(model.q - qmin))
prb.createResidual("q_limit_upper", 100 * utils.barrier1(model.q - qmax))

cd_fun = model.kd.computeCentroidalDynamics()
h_lin, h_ang, dh_lin, dh_ang = cd_fun(model.q, model.v, model.a)
# prb.createIntermediateResidual('min_angular_mom', 1e-1 * dh_ang)

rospy.set_param('/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('cogimon_controller') + '/config/cogimon_straight_config.yaml')

def cop_lims_fun(model:FullModelInverseDynamics, frame:str):
    fl = 0.23
    hl = 0.06
    al = 0.1
    tl = 0.03
    eps = 0.01
    K = 500

    q = cs.SX.sym('q', model.nq)

    # compute pitch angle of the foot
    R = model.kd.fk(frame)(q=q)['ee_rot']
    theta = cs.atan2(-R[2, 0], cs.sqrt(R[0, 0]**2 + R[1, 0]**2))

    min = -hl + (al + hl) * 1 / (1 + cs.exp(-K * (theta - 0.01)))
    # min = hl + eps + (-al - hl) * 1 / (1 + cs.exp(-K * (theta - 0.004)))
    max = -hl + eps + (al+tl + hl-eps) * 1 / (1 + cs.exp(-K * (theta + 0.01)))
    # max = hl + (-fl+hl - hl) * 1 / (1 + cs.exp(-K * (theta + 0.004)))

    input_list = [q]
    output_list = [min, max]

    f = cs.Function('cop_lims', input_list, output_list, ['q'], ['min', 'max'])
    return f

for frame, wrench in model.getForceMap().items():
    cop_lims = cop_lims_fun(model, frame)
    c_rot = model.kd.fk(frame)(q=model.q)['ee_rot']
    f_local = c_rot.T @ wrench[:3]
    t_local = c_rot.T @ wrench[3:]
    prb.createIntermediateResidual(f'min_pitch_cop_{frame}', 100 * utils.barrier1(t_local[1] + f_local[2] * cop_lims(q=model.q)['min']))
    prb.createIntermediateResidual(f'max_pitch_cop_{frame}', 100 * utils.barrier1(-t_local[1] - f_local[2] * cop_lims(q=model.q)['max']))


'''
Foot vertices relative distance constraint
'''
pos_lf = model.kd.fk('l_sole')(q=model.q)['ee_pos']
pos_rf = model.kd.fk('r_sole')(q=model.q)['ee_pos']
base_ori = model.kd.fk('base_link')(q=model.q)['ee_rot']
rel_dist = base_ori.T @ (pos_lf - pos_rf)

# prb.createResidual('relative_distance_lower_x', utils.barrier(rel_dist[0] + 0.3))
# prb.createResidual('relative_distance_upper_x', utils.barrier1(rel_dist[0] - 0.4))
prb.createResidual('relative_distance_lower_y', 100. * utils.barrier(rel_dist[1] - 0.15))
# prb.createResidual('relative_distance_upper_y', 100. * utils.barrier1(rel_dist[1] - 0.35))

weight = np.empty([1, 6])
w_force = 0.
w_torque = 10.0
weight = np.atleast_2d(np.array([w_force, w_force, w_force, w_torque, w_torque, w_torque]))
for contact, force in model.getForceMap().items():
    prb.createIntermediateResidual(f'force_regularization_{contact}', weight @ force)

tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns + 1)
# phase manager handling
c_timelines = dict()
for c in model.cmap:
    c_timelines[c] = pm.createTimeline(f'{c}_timeline')

for c in model.cmap:
    # stance phase
    stance_duration = 12 * scale
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}')
    stance_phase.addItem(ti.getTask(f'foot_contact_{c}'))
    pitch_ref = np.zeros(shape=[7, stance_duration - 4])
    pitch_ref[6, :] = np.ones(shape=[1, stance_duration - 4])
    stance_phase.addItemReference(ti.getTask(f'pitch_{c}'), pitch_ref, nodes=range(2, stance_duration-2))

    short_stance_duration = 2 * scale
    short_stance_phase = c_timelines[c].createPhase(short_stance_duration, f'short_stance_{c}')
    short_stance_phase.addItem(ti.getTask(f'foot_contact_{c}'))

    flight_duration = 12 * scale
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.05, [None, 0, None]))
    flight_phase.addItemReference(ti.getTask(f'foot_z_{c}'), ref_trj)

def step(swing, stance):
    c_timelines[swing].addPhase(c_timelines[swing].getRegisteredPhase(f'flight_{swing}'))
    c_timelines[stance].addPhase(c_timelines[stance].getRegisteredPhase(f'stance_{stance}'))
    c_timelines[stance].addPhase(c_timelines[stance].getRegisteredPhase(f'short_stance_{stance}'))
    c_timelines[swing].addPhase(c_timelines[swing].getRegisteredPhase(f'short_stance_{swing}'))

def stand():
    for c in model.cmap:
        stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
        c_timelines[c].addPhase(stance)


stand()
step('l_sole', 'r_sole')
step('r_sole', 'l_sole')
step('l_sole', 'r_sole')
step('r_sole', 'l_sole')
step('l_sole', 'r_sole')
# step('r_sole', 'l_sole')
# step('l_sole', 'r_sole')
# step('r_sole', 'l_sole')

for c in model.cmap:
    while c_timelines[c].getEmptyNodes() > 0:
        stand()

ti.getTask('final_base_xy').setRef(np.atleast_2d([1.5, 0, 0, 0, 0, 0, 0]).T)

com_vel = model.kd.centerOfMass()(q=model.q, v=model.v, a=model.a)['vcom']
sum_f = 0
for cname, cforce in model.getForceMap().items():
    rot = model.kd.fk(cname)(q=model.q)['ee_rot']
    w_force = cs.mtimes(rot.T, cforce[:3])
    sum_f += w_force
    # sum_f += cforce[:3]

W_com = cs.mtimes(sum_f.T, com_vel)
W_com = sum_f.T @ com_vel
prb.createIntermediateResidual('min_com_power', 0.5 * W_com)

J_l = model.kd.jacobian('l_sole', model.kd_frame)(q=model.q)['J']
J_r = model.kd.jacobian('r_sole', model.kd_frame)(q=model.q)['J']

prb.createResidual('singularity_left', 100 * utils.barrier(cs.sqrt(cs.det(cs.mtimes(J_l, J_l.T))) - 0.1))
prb.createResidual('singularity_right', 100 * utils.barrier(cs.sqrt(cs.det(cs.mtimes(J_r, J_r.T))) - 0.1))

model.q.setBounds(model.q0, model.q0, nodes=0)
model.q[3:18].setBounds(model.q0[3:18], model.q0[3:18], nodes=ns)
model.v.setBounds(model.v0, model.v0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=ns)

model.q.setInitialGuess(model.q0)
model.v.setInitialGuess(model.v0)

f0 = [0, 0, kin_dyn.mass() / 8 * 9.8, 0, 0, 0]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

# for contact, force in model.getForceMap().items():
#     force_z_ref[contact].assign(f0[2])

# finalize taskInterface and solve bootstrap problem

ti.finalize()
ti.bootstrap()

solution = ti.solution

import math
cop_min_l_sole = list()
cop_max_l_sole = list()
cop_min_r_sole = list()
cop_max_r_sole = list()

theta_l_sole = list()
theta_r_sole = list()
rot_force_l_sole = np.empty(shape=[6, solution['q'].shape[1] + 1])

for i in range(solution['a'].shape[1]):
    q = solution['q'][:, i]
    cop_lims_l_sole = cop_lims_fun(model, 'l_sole')
    cop_lims_r_sole = cop_lims_fun(model, 'r_sole')
    cop_min_l_sole.append(cop_lims_l_sole(q=q)['min'])
    cop_max_l_sole.append(cop_lims_l_sole(q=q)['max'])
    cop_min_r_sole.append(cop_lims_r_sole(q=q)['min'])
    cop_max_r_sole.append(cop_lims_r_sole(q=q)['max'])

    R_l_sole = model.kd.fk('l_sole')(q=solution['q'][:, i])['ee_rot']
    theta_l_sole.append(math.atan2(-R_l_sole[2, 0], math.sqrt(R_l_sole[0, 0] ** 2 + R_l_sole[1, 0] ** 2)))
    rot_force_l_sole[:, i] = np.vstack([R_l_sole.T @ solution['f_l_sole'][:3, i], R_l_sole.T @ solution['f_l_sole'][3:, i]]).flatten()

    R_r_sole = model.kd.fk('r_sole')(q=solution['q'][:, i])['ee_rot']
    theta_r_sole.append(math.atan2(-R_r_sole[2, 0], math.sqrt(R_r_sole[0, 0] ** 2 + R_r_sole[1, 0] ** 2)))



import matplotlib.pyplot as plt
cop_min_l_sole_list = [elem.elements()[0] for elem in cop_min_l_sole]
cop_min_r_sole_list = [elem.elements()[0] for elem in cop_min_r_sole]
cop_max_l_sole_list = [elem.elements()[0] for elem in cop_max_l_sole]
cop_max_r_sole_list = [elem.elements()[0] for elem in cop_max_r_sole]

fig, a = plt.subplots()
a.plot(cop_min_l_sole_list)
a.plot(cop_max_l_sole_list)
a.plot(theta_l_sole)
a.grid()

fig2, axs = plt.subplots(nrows=3, ncols=1)
axs[0].plot(rot_force_l_sole[4, :])
axs[1].plot(rot_force_l_sole[0, :], label='Fx')
axs[1].plot(rot_force_l_sole[1, :], label='Fy')
axs[1].plot(rot_force_l_sole[2, :], label='Fz')
axs[2].plot(solution['f_l_sole'][0, :], label='Fx')
axs[2].plot(solution['f_l_sole'][1, :], label='Fy')
axs[2].plot(solution['f_l_sole'][2, :], label='Fz')
for i in range(solution['f_l_sole'].shape[1]):
    axs[2].arrow(x=i, y=0, dx=solution['f_l_sole'][0, i], dy=solution['f_l_sole'][2, i], color=[0.658, 0, 0])
axs[1].legend()
axs[2].legend()
axs[0].grid()
axs[1].grid()
axs[2].grid()
plt.show()

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(prb.getDt(), kin_dyn.joint_names(), solution['q'], kindyn=kin_dyn, trajectory_markers=contact_list_repl)

if robot is not None:
    play_on_robot(solution, model)
else:
    repl.replay()



