#!/usr/bin/python3

import horizon.utils.kin_dyn as kd
from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from horizon.ros import replay_trajectory
from horizon.utils.resampler_trajectory import Resampler
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
from sensor_msgs.msg import Joy
import cartesian_interface.roscpp_utils as roscpp
import horizon.utils.analyzer as analyzer

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

from scipy.spatial.transform import Rotation
import colorama
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
# from kyon_controller.msg import WBTrajectory

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import time

import horizon.utils as utils

global base_pose
global base_twist


def gt_pose_callback(msg):
    global base_pose
    base_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                          msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                          msg.pose.orientation.w])


def gt_twist_callback(msg):
    global base_twist
    base_twist = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                           msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])


def set_state_from_robot(robot_joint_names, q_robot, qdot_robot, fixed_joint_map={}):
    robot.sense()

    # manage fixed joints if any
    q_map = robot.getJointPositionMap()

    for fixed_joint in fixed_joint_map:
        if fixed_joint in q_map:
            del q_map[fixed_joint]

    q_index = 0
    for j_name in robot_joint_names:
        q_robot[q_index] = q_map[j_name]
        q_index += 1

    # numerical problem: two quaternions can represent the same rotation
    # if difference between the base orientation in the state x and the sensed one base_pose < 0, change sign
    state_quat_conjugate = np.copy(x_opt[3:7, 0])
    state_quat_conjugate[:3] *= -1.0

    # normalize the quaternion
    state_quat_conjugate = state_quat_conjugate / np.linalg.norm(x_opt[3:7, 0])
    diff_quat = _quaternion_multiply(base_pose[3:], state_quat_conjugate)

    if diff_quat[3] < 0:
        base_pose[3:] = -base_pose[3:]

    q = np.hstack([base_pose, q_robot])
    model.q.setBounds(q, q, nodes=0)

    qdot = robot.getJointVelocity()
    qdot_map = robot.eigenToMap(qdot)

    for fixed_joint in fixed_joint_map:
        if fixed_joint in qdot_map:
            del qdot_map[fixed_joint]

    qdot_index = 0
    for j_name in robot_joint_names:
        qdot_robot[qdot_index] = qdot_map[j_name]
        qdot_index += 1

    # VELOCITY OF PINOCCHIO IS LOCAL, BASE_TWIST FROM  XBOTCORE IS GLOBAL:
    # transform it in local
    r_base = Rotation.from_quat(base_pose[3:]).as_matrix()

    r_adj = np.zeros([6, 6])
    r_adj[:3, :3] = r_base.T
    r_adj[3:6, 3:6] = r_base.T

    # rotate in the base frame the relative velocity (ee_v_distal - ee_v_base_distal)
    ee_rel = r_adj @ base_twist

    qdot = np.hstack([ee_rel, qdot_robot])
    model.v.setBounds(qdot, qdot, nodes=0)




rospy.init_node('concert_receding')
roscpp.init('concert_receding', [])

# solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)
# rospy.sleep(1.)

'''
Load urdf and srdf
'''

with open("modularbot.urdf", "rb") as f:
    urdf = f.read().decode("UTF-8")

# get from ros param the urdf and srdf

# urdf = rospy.get_param(param_name='/robot_description', default='')
# if urdf == '':
#     raise print('urdf not set')
#
# srdf = rospy.get_param(param_name='/robot_description_semantic', default='')
# if srdf == '':
#     raise print('srdf not set')

file_dir = os.getcwd()

'''
Initialize Horizon problem
'''
ns = 30
T = 1.5
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

'''
Build ModelInterface and RobotStatePublisher
'''
# cfg = co.ConfigOptions()
# cfg.set_urdf(urdf)
# cfg.set_srdf(srdf)
# cfg.generate_jidmap()
# cfg.set_string_parameter('model_type', 'RBDL')
# cfg.set_string_parameter('framework', 'ROS')
# cfg.set_bool_parameter('is_model_floating_base', True)
#
robot = None

# try:
#     robot = xbot.RobotInterface(cfg)
#     rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)
#     rospy.Subscriber('/xbotcore/link_state/pelvis/twist', TwistStamped, gt_twist_callback)
#     while base_pose is None or base_twist is None:
#         rospy.sleep(0.01)
#     robot.sense()
#     q_init = robot.getPositionReference()
    # q_init = robot.getJointPosition()
    # q_init = robot.eigenToMap(q_init)
#
# except:
print('RobotInterface not created')
q_init = {'J1_A': 0.0,
          'J_wheel_A': 0.0,
          'J1_B': 0.0,
          'J_wheel_B': 0.0,
          'J1_C': 0.0,
          'J_wheel_C': 0.0,
          'J1_D': 0.0,
          'J_wheel_D': 0.0,
          'J1_E': 0.0,
          'J2_E': -0.5,
          'J3_E': 0.0,
          'J4_E': 0.5,
          'J5_E': 0.0,
          'J6_E': -0.5
          }

base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

FK = kin_dyn.fk('J_wheel_A')
init = base_init.tolist() + list(q_init.values())
init_pos_foot = FK(q=kin_dyn.mapToQ(q_init))['ee_pos']
base_init[2] = -init_pos_foot[2]

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init
                                 )

rospy.set_param('mpc/robot_description', urdf)
bashCommand = 'rosrun robot_state_publisher robot_state_publisher robot_description:=mpc/robot_description'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)

# ti.setTaskFromYaml(rospkg.RosPack().get_path('kyon_controller') + '/config/concert_config.yaml')
ti.setTaskFromYaml('/home/francesco/catkin_ws/external/kyon_controller/config/concert_config.yaml')

pm = pymanager.PhaseManager(ns)
# phase manager handling
c_phases = dict()
for c in model.cmap.keys():
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

# weight more roll joints
white_list_indices = list()
black_list_indices = list()
white_list = []

black_list = ['J_wheel_A', 'J_wheel_B', 'J_wheel_C', 'J_wheel_D']


postural_joints = np.array(list(range(7, model.nq)))
for joint in black_list:
    black_list_indices.append(model.joint_names.index(joint))
for joint in white_list:
    white_list_indices.append(7 + model.joint_names.index(joint))
postural_joints = np.delete(postural_joints, black_list_indices)

if white_list:
    prb.createResidual("min_q_white_list", 5. * (model.q[white_list_indices] - model.q0[white_list_indices]))
# if black_list:
#     prb.createResidual('velocity_regularization', 0.1 * model.v[postural_joints])

# stance_duration = 8
# for c in model.cmap.keys():
    # stance phase normal
    # stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    # if ti.getTask(f'{c}_contact') is not None:
    #     stance_phase.addItem(ti.getTask(f'{c}_contact'))
    # else:
    #     raise Exception('task not found')
    #
    # c_phases[c].registerPhase(stance_phase)

# for c in model.cmap.keys():
#     stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
#     while c_phases[c].getEmptyNodes() > 0:
#         c_phases[c].addPhase(stance)

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
# ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
# ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)

vel_lims = model.kd.velocityLimits()
prb.createResidual('max_vel', 1e1 * utils.utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))

# finalize taskInterface and solve bootstrap problem
ti.finalize()

ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())
repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(), np.array([]),
                                           {k: None for k in model.fmap.keys()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)
                                           # future_trajectory_markers={'base_link': 'world', 'ball_1': 'world'})

global joy_msg

xig = np.empty([prb.getState().getVars().shape[0], 1])
time_elapsed_shifting_list = list()
time_elapsed_solving_list = list()
time_elapsed_all_list = list()

from concert_joy_commands import GaitManager, JoyCommands
contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)

jc = JoyCommands(gm)


# if 'wheel_joint_1' in model.kd.joint_names():
#     from geometry_msgs.msg import PointStamped
#     zmp_pub = rospy.Publisher('zmp_pub', PointStamped, queue_size=10)

# anal = analyzer.ProblemAnalyzer(prb)

# import matplotlib.pyplot as plt
# plt.ion()  # Turn on interactive mode
# fig, ax = plt.subplots()
# line, = ax.plot(range(prb.getNNodes() - 1), ti.solver_bs.getConstraintsValues()['dynamics'][0, :])  # Plot initial data
# ax.set_ylim(-2., 2.)  # Set your desired limits here

def _quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])

robot_joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

q_robot = np.zeros(len(robot_joint_names))
qdot_robot = np.zeros(len(robot_joint_names))

# while not rospy.is_shutdown():
while not rospy.is_shutdown():
    # tic = time.time()
    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]

    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    # closed loop
    if robot is not None:
        set_state_from_robot(robot_joint_names=robot_joint_names, q_robot=q_robot, qdot_robot=qdot_robot)

        # print("base_pose: ", base_pose)
    # shift phases of phase manager
    tic = time.time()
    pm.shift()
    time_elapsed_shifting = time.time() - tic
    time_elapsed_shifting_list.append(time_elapsed_shifting)

    jc.run(solution)

    tic = time.time()
    ti.rti()
    time_elapsed_solving = time.time() - tic
    time_elapsed_solving_list.append(time_elapsed_solving)

    # line.set_ydata(ti.solver_rti.getConstraintsValues()['dynamics'][0, :])
    # #
    # ax.relim()  # Update the limits of the axes
    # ax.autoscale_view()  # Autoscale the axes view
    # fig.canvas.draw()
    # fig.canvas.flush_events()
    #
    # plt.pause(0.0001) # Add a small delay to see the changes



    # for elem_name, elem_values in ti.solver_rti.getConstraintsValues().items():
    #     print(f"{colorama.Fore.GREEN}{elem_name}:  {elem_values}{colorama.Fore.RESET}")

    # for elem_name, elem_values in ti.solver_rti.getCostsValues().items():
    #     print(f"{colorama.Fore.RED}{elem_name}:  {elem_values}{colorama.Fore.RESET}")

    solution = ti.solution

    # ================ SOLUTION MESSAGE FOR CONTROLLER =================
    # sol_msg = WBTrajectory()
    # sol_msg.header.frame_id = 'world'
    # sol_msg.header.stamp = rospy.Time.now()
    #
    # sol_msg.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

    # sol_msg.q = solution['q'][:, 0].tolist()
    # sol_msg.v = solution['v'][:, 0].tolist()
    # sol_msg.q = solution['q'][:, 1].tolist()
    # sol_msg.v = solution['v'][:, 1].tolist()
    # sol_msg.a = solution['a'][:, 0].tolist()

    # for frame in model.getForceMap():
    #     sol_msg.force_names.append(frame)
    #     sol_msg.f.append(
    #         Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))
    #
    # solution_publisher.publish(sol_msg)

    # replay stuff
    # if robot is None:
    repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
    repl.publish_joints(solution['q'][:, 0])
    # repl.publish_joints(solution['q'][:, ns], prefix='last')
    repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)
    # repl.publish_future_trajectory_marker('base_link', solution['q'][0:3, :])
    # repl.publish_future_trajectory_marker('ball_1', solution['q'][8:11, :])

    time_elapsed_all = time.time() - tic
    time_elapsed_all_list.append(time_elapsed_all)

    rate.sleep()



    # print(f"{colorama.Style.RED}MPC loop elapsed time: {time.time() - tic}{colorama.Style.RESET}")

print(f'average time elapsed shifting: {sum(time_elapsed_shifting_list) / len(time_elapsed_shifting_list)}')
print(f'average time elapsed solving: {sum(time_elapsed_solving_list) / len(time_elapsed_solving_list)}')
print(f'average time elapsed all: {sum(time_elapsed_all_list) / len(time_elapsed_all_list)}')