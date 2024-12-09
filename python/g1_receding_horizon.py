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
import phase_manager.pyrosserver as pyrosserver

from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS

from urdf_augment import URDFAugment
from sensor_msgs.msg import Joy
from cogimon_controller.msg import WBTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PointStamped

from matplotlib import pyplot as hplt
import time
from matlogger2 import matlogger

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import colorama


global base_pose
global base_twist

'''
joystick interface
'''
joystick_flag = rospy.get_param(param_name='~joy', default=True)

'''
open/closed loop
'''
closed_loop = rospy.get_param(param_name='~closed_loop', default=False)

'''
xbot
'''
xbot_param = rospy.get_param(param_name="~xbot", default=False)

def gt_pose_callback(msg):
    global base_pose
    base_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                          msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                          msg.pose.orientation.w])


def gt_twist_callback(msg):
    global base_twist
    base_twist = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                           msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])


def joy_callback(msg):
    global joy_msg
    joy_msg = msg

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def conjugate_quaternion(q):
    q_conjugate = np.copy(q)
    q_conjugate[1:] *= -1.0
    return q_conjugate
def rotate_vector(vector, quaternion):
    # normalize the quaternion
    quaternion = quaternion / np.linalg.norm(quaternion)

    # construct a pure quaternion
    v = np.array([0, vector[0], vector[1], vector[2]])

    # rotate the vector p = q* v q
    rotated_v = quaternion_multiply(quaternion,
                                          quaternion_multiply(v, conjugate_quaternion(quaternion)))

    # extract the rotated vector
    rotated_vector = rotated_v[1:]

    return rotated_vector

def incremental_rotate(q_initial: np.quaternion, d_angle, axis) -> np.quaternion:
    # np.quaternion is [w,x,y,z]
    q_incremental = np.array([np.cos(d_angle / 2),
                              axis[0] * np.sin(d_angle / 2),
                              axis[1] * np.sin(d_angle / 2),
                              axis[2] * np.sin(d_angle / 2)
                              ])

    # normalize the quaternion
    q_incremental /= np.linalg.norm(q_incremental)

    # initial orientation of the base

    # final orientation of the base
    q_result = np.quaternion(*q_incremental) * np.quaternion(*q_initial)

    return q_result


rospy.init_node('g1_walk')

solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=10)

rospy.Subscriber('/joy', Joy, joy_callback)

'''
Load urdf and srdf
'''
g1_description_folder = rospkg.RosPack().get_path('cogimon_controller')
urdf_path = g1_description_folder + "/urdf/g1_29dof.urdf"
srdf_path = g1_description_folder + "/srdf/g1_29dof.srdf"

srdf = open(srdf_path, 'r').read()
urdf_aug = URDFAugment(urdf_path)

sole_xy = [0.15, 0.05]
urdf_aug.addReferenceFrame('l_sole', 'left_ankle_roll_link', origin_xyz=[0, 0, -0.03])
urdf_aug.addReferenceFrame('r_sole', 'right_ankle_roll_link', origin_xyz=[0, 0, -0.03])
urdf_aug.addRectangleReferenceFrame('l_sole', size=sole_xy, offset_x=0.03)
urdf_aug.addRectangleReferenceFrame('r_sole', size=sole_xy, offset_x=0.03)
urdf = urdf_aug.getXml()

rospy.set_param('/robot_description', urdf)

# urdf = rospy.get_param('/robot_description', "")
# if urdf == "":
#     raise print("missing mandatory parameter 'urdf'")
# srdf = rospy.get_param('/robot_description_semantic', "")
# if srdf == "":
#     raise print("missing mandatory parameter 'srdf'")

# file_dir = os.getcwd()


#
base_pose = None
base_twist = None

if xbot_param:
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'Unitree')
    cfg.set_bool_parameter('is_model_floating_base', True)
    robot = xbot.RobotInterface(cfg)

    base_pose = np.array([0.0, 0.0, 0.0, 0., 0.0, 0.0, 1.0])
    base_twist = np.zeros(6)

    robot.sense()
    q_init = robot.getPositionReference()
    q_init = robot.eigenToMap(q_init)

    q_init = {'left_hip_pitch_joint': -0.15,
              'left_hip_roll_joint': 0.,
              'left_hip_yaw_joint': 0.,
              'left_knee_joint': 0.45,
              'left_ankle_pitch_joint': -0.3,
              'left_ankle_roll_joint': 0.,
              'right_hip_pitch_joint': -0.15,
              'right_hip_roll_joint': 0.,
              'right_hip_yaw_joint': 0.,
              'right_knee_joint': 0.45,
              'right_ankle_pitch_joint': -0.3,
              'right_ankle_roll_joint': 0.,
              'waist_yaw_joint': 0.,
              'waist_roll_joint': 0.,
              'waist_pitch_joint': 0.,
              'left_shoulder_pitch_joint': 0.,
              'left_shoulder_roll_joint': 0.,
              'left_shoulder_yaw_joint': 0.,
              'left_elbow_joint': 0.9,
              'left_wrist_roll_joint': 0.,
              'left_wrist_pitch_joint': 0.,
              'left_wrist_yaw_joint': 0.,
              'right_shoulder_pitch_joint': 0.,
              'right_shoulder_roll_joint': 0.,
              'right_shoulder_yaw_joint': 0.,
              'right_elbow_joint': 0.9,
              'right_wrist_roll_joint': 0.,
              'right_wrist_pitch_joint': 0.,
              'right_wrist_yaw_joint': 0.}
    
    print(colorama.Fore.CYAN + 'RobotInterface created!' + colorama.Fore.RESET)

else:
    print(colorama.Fore.CYAN + 'RobotInterface not created' + colorama.Fore.RESET)
    base_pose = np.array([0.0, 0.0, 0.0, 0., 0.0, 0.0, 1.0])
    base_twist = np.zeros(6)
    robot = None

    q_init = {'floating_base_joint': 0.,
              'left_hip_pitch_joint': -0.15,
              'left_hip_roll_joint': 0.,
              'left_hip_yaw_joint': 0.,
              'left_knee_joint': 0.45,
              'left_ankle_pitch_joint': -0.3,
              'left_ankle_roll_joint': 0.,
              'right_hip_pitch_joint': -0.15,
              'right_hip_roll_joint': 0.,
              'right_hip_yaw_joint': 0.,
              'right_knee_joint': 0.45,
              'right_ankle_pitch_joint': -0.3,
              'right_ankle_roll_joint': 0.,
              'waist_yaw_joint': 0.,
              'waist_roll_joint': 0.,
              'waist_pitch_joint': 0.,
              'left_shoulder_pitch_joint': 0.,
              'left_shoulder_roll_joint': 0.,
              'left_shoulder_yaw_joint': 0.,
              'left_elbow_joint': 0.9,
              'left_wrist_roll_joint': 0.,
              'left_wrist_pitch_joint': 0.,
              'left_wrist_yaw_joint': 0.,
              'right_shoulder_pitch_joint': 0.,
              'right_shoulder_roll_joint': 0.,
              'right_shoulder_yaw_joint': 0.,
              'right_elbow_joint': 0.9,
              'right_wrist_roll_joint': 0.,
              'right_wrist_pitch_joint': 0.,
              'right_wrist_yaw_joint': 0.}


l_foot_link_name = 'l_sole'
r_foot_link_name = 'r_sole'
base_link_name = 'pelvis'

'''
Initialize Horizon problem
'''
# ns = 30
# T = 1.5
ns = 30
T = 1.5
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf) #, fixed_joints=fixed_joint_map)

# setting base of the robot
FK = kin_dyn.fk('l_sole')
init = base_pose.tolist() + list(q_init.values())
init_pos_foot = FK(q=kin_dyn.mapToQ(q_init))['ee_pos']
base_pose[2] = -init_pos_foot[2]

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose)
                                 # fixed_joint_map=fixed_joint_map)


# rospy.set_param('/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)

ti.setTaskFromYaml(rospkg.RosPack().get_path('cogimon_controller') + '/config/g1_config.yaml')


qmin = np.array(model.kd.q_min())
qmax = np.array(model.kd.q_max())
prb.createResidual("q_limit_lower", 100 * utils.barrier(model.q - qmin))
prb.createResidual("q_limit_upper", 100 * utils.barrier1(model.q - qmax))

'''
adding minimization of angular momentum
'''
cd_fun = ti.model.kd.computeCentroidalDynamics()
h_lin, h_ang, dh_lin, dh_ang = cd_fun(model.q, model.v, model.a)
prb.createIntermediateResidual('min_angular_mom', 1e-1 * dh_ang)

'''
Foot vertices relative distance constraint
'''
pos_lf = model.kd.fk(l_foot_link_name)(q=model.q)['ee_pos']
pos_rf = model.kd.fk(r_foot_link_name)(q=model.q)['ee_pos']
base_ori = model.kd.fk(base_link_name)(q=model.q)['ee_rot']
rel_dist = base_ori.T @ (pos_lf - pos_rf)

# prb.createResidual('relative_distance_lower_x', utils.barrier(rel_dist[0] + 0.3))
# prb.createResidual('relative_distance_upper_x', utils.barrier1(rel_dist[0] - 0.4))
prb.createResidual('relative_distance_lower_y', 100. * utils.barrier(rel_dist[1] - 0.15))
# prb.createResidual('relative_distance_upper_y', 10. * utils.barrier1(rel_dist[1] - 0.35))kl.

# force_z_ref = dict()
# for contact, force in model.getForceMap().items():
#     print(f'{contact}: {force}')
#     force_z_ref[contact] = prb.createParameter(f'{contact}_force_z_ref', 1)
#     prb.createIntermediateResidual(f'{contact}_force_z_reg', 0.001 * (force[2] - force_z_ref[contact]))

# com_vel = model.kd.centerOfMass()(q=model.q, v=model.v, a=model.a)['vcom']
# sum_f = 0
# for cname, cforce in model.getForceMap().items():
#     rot = model.kd.fk(cname)(q=model.q)['ee_rot']
#     w_force = cs.mtimes(rot.T, cforce[:3])
#     sum_f += w_force
#     # sum_f += cforce[:3]
#
# W_com = cs.mtimes(sum_f.T, com_vel)
# W_com = sum_f.T @ com_vel
# prb.createIntermediateResidual('min_com_power', 0.5 * W_com)


tg = trajectoryGenerator.TrajectoryGenerator()

pm = pymanager.PhaseManager(ns + 1)
# phase manager handling
c_timelines = dict()


contact_task_dict = {'l_sole': 'foot_contact_l',
                     'r_sole': 'foot_contact_r'}

z_task_dict = {'l_sole': 'foot_z_l',
               'r_sole': 'foot_z_r'}

# MAP -> contact name : timeline
contact_phase_map = dict()
for c in model.getContactMap():
    c_timelines[c] = pm.createTimeline(f'{contact_task_dict[c]}_timeline')

    contact_phase_map[c] = f'{contact_task_dict[c]}_timeline'

stance_duration = 10
short_stance_duration = 4
flight_duration = 10

for c in model.getContactMap():
    # stance phase
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{contact_task_dict[c]}')
    stance_phase.addItem(ti.getTask(contact_task_dict[c]))

    short_stance_phase = c_timelines[c].createPhase(short_stance_duration, f'short_stance_{contact_task_dict[c]}')
    short_stance_phase.addItem(ti.getTask(contact_task_dict[c]))

    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{contact_task_dict[c]}')
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot, 0.05, [None, 0, None]))
    flight_phase.addItemReference(ti.getTask(z_task_dict[c]), ref_trj)
    # flight_phase.addItem(ti.getTask(f'zero_vel_xy_{c}'))

for c in model.cmap:
    stance = c_timelines[c].getRegisteredPhase(f'stance_{contact_task_dict[c]}')
    flight = c_timelines[c].getRegisteredPhase(f'flight_{contact_task_dict[c]}')
    short_stance = c_timelines[c].getRegisteredPhase(f'short_stance_{contact_task_dict[c]}')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)

model.q.setBounds(model.q0, model.q0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=ns)
model.q.setInitialGuess(ti.model.q0)

f0 = [0, 0, kin_dyn.mass() / 8 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

# finalize taskInterface and solve bootstrap problem
ti.finalize()


pm.update()
rs = pyrosserver.RosServerClass(pm)

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()
time_elapsed_solving_list = list()
time_elapsed_all_list = list()

gm = GaitManager(ti, pm, contact_phase_map)

if joystick_flag:
    from joy_commands import JoyCommands
    jc = JoyCommands()

gait_manager_ros = GaitManagerROS(gm)

robot_joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'floating_base_joint']]


while not rospy.is_shutdown():
    tic = time.time()
    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]

    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    pm.shift()

    # publishes to ros phase manager info
    rs.run()

    if joystick_flag:
        # receive msgs from joystick and publishes to ROS topic
        jc.run()

    # receive msgs from ros topic and send commands to robot
    gait_manager_ros.run()
    pm.update()

    ti.rti()
    solution = ti.solution
    # for force_name in model.getForceMap():
    #     logger.add(force_name, solution[f'f_{force_name}'][:, 0])

    sol_msg = WBTrajectory()
    sol_msg.header.frame_id = 'world'
    sol_msg.header.stamp = rospy.Time.now()

    sol_msg.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'floating_base_joint']]

    sol_msg.q = solution['q'][:, 0].tolist()
    sol_msg.v = solution['v'][:, 0].tolist()
    sol_msg.a = solution['a'][:, 0].tolist()

    for frame in model.getForceMap():
        sol_msg.force_names.append(frame)
        sol_msg.f.append(
            Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))

    solution_publisher.publish(sol_msg)

    # replay stuff
    if robot is None:
        repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
        repl.publish_joints(solution['q'][:, 0])
        repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)

    # time_elapsed_all = time.time() - tic

    rate.sleep()






















