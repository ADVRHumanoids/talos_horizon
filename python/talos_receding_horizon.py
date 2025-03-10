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
from horizon.rhc.gait_manager import PhaseGaitWrapper
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS

from urdf_augment import URDFAugment
from sensor_msgs.msg import Joy, JointState
from talos_horizon.msg import WBTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PointStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from matplotlib import pyplot as hplt
import time
from matlogger2 import matlogger
from scipy.spatial.transform import Rotation

import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
import colorama

global base_pose
global base_twist
global b_T_imu
global joint_read
global joint_cmd
global joint_names

def gt_pose_callback(msg):
    global base_pose
    base_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                          msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                          msg.pose.orientation.w])


def gt_twist_callback(msg):
    global base_twist
    base_twist = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                           msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

def imu_callback(msg:Imu):
    global base_pose
    global base_twist
    base_pose = np.array([0., 0., 0., msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    base_twist = np.array([0., 0., 0., msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    # rotate in base_link frame
    w_T_imu = Affine3(rot=base_pose[3:])
    w_T_b = w_T_imu * b_T_imu.inverse()
    base_pose[3:] = w_T_b.quaternion
    base_pose[3:] = base_pose[3:] / np.linalg.norm(base_pose[3:])
    base_twist[3:] = b_T_imu.linear @ base_twist[3:]

def joint_read_callback(msg: JointState):
    global joint_read
    joint_read = msg.position

def joint_cmd_callback(msg: JointState):
    global joint_cmd
    global joint_names
    joint_cmd = msg.position
    joint_names = msg.name


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
    state_quat_conjugate = np.copy(base_pose[3:])
    state_quat_conjugate[:3] *= -1.0

    # normalize the quaternion
    state_quat_conjugate = state_quat_conjugate / np.linalg.norm(base_pose[3:])
    diff_quat = utils.quaternion_multiply(base_pose[3:], state_quat_conjugate)

    if diff_quat[3] < 0:
        base_pose[3:] = -base_pose[3:]

    q = np.hstack([base_pose, q_robot])
    model.q[3:].setBounds(q[3:], q[3:], nodes=0)

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
    model.v.setBounds(np.hstack([[-np.inf, -np.inf, -np.inf], qdot[3:]]),
                      np.hstack([[np.inf, np.inf, np.inf], qdot[3:]]), nodes=0)


rospy.init_node('talos_walk')

solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=10)

rospy.Subscriber('/joy', Joy, joy_callback)

'''
Load ros params
'''
joystick_flag = rospy.get_param(param_name='~joy', default=True)
closed_loop = rospy.get_param(param_name='~closed_loop', default=False)
xbot_param = rospy.get_param(param_name="~xbot", default=False)
talos_param = rospy.get_param(param_name="~talos", default=False)

closed_loop = False

if talos_param:
    solution_publisher = rospy.Publisher('/inria_controller/sub_joint_cmd', JointState, queue_size=10)
else:
    solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=10)

time_publisher = rospy.Publisher('/mpc_node_loop_time', Float32MultiArray, queue_size=10)

# if closed_loop:
#     xbot_param = True

'''
Load urdf and srdf
'''
talos_description_folder = rospkg.RosPack().get_path('talos_cartesio_config')
urdf_path = talos_description_folder + "/robots/talos.urdf"
srdf_path = talos_description_folder + "/robots/talos.srdf"

srdf = open(srdf_path, 'r').read()

'''
Augment urdf with feet reference frames
'''

urdf_aug = URDFAugment(urdf_path)

sole_xy = [0.2, 0.1]
urdf_aug.addReferenceFrame('left_contact_link', 'leg_left_6_link', origin_xyz=[0, 0, -0.107])
urdf_aug.addReferenceFrame('right_contact_link', 'leg_right_6_link', origin_xyz=[0, 0, -0.107])
urdf_aug.addRectangleReferenceFrame('left_contact_link', size=sole_xy)
urdf_aug.addRectangleReferenceFrame('right_contact_link', size=sole_xy)
urdf = urdf_aug.getXml()

rospy.delete_param('/robot_description')
rospy.set_param('/robot_description', urdf)

robot = None
base_pose = None
base_twist = None
b_T_imu = None
joint_read = None
joint_cmd = None

if xbot_param:
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', True)
    model_xbot = xbot.ModelInterface(cfg)
    robot = xbot.RobotInterface(cfg)

    b_T_imu = model_xbot.getPose('imu_link', 'base_link')

    if closed_loop:
        rospy.Subscriber("/xbotcore/link_state/base_link/pose", PoseStamped, gt_pose_callback)
        rospy.Subscriber("/xbotcore/link_state/base_link/twist", TwistStamped, gt_twist_callback)
        # rospy.Subscriber("/xbotcore/imu/imu_link", Imu, imu_callback)
        while base_pose is None or base_twist is None:
            print('Waiting for imu data')
            rospy.sleep(0.01)

    else:
        base_pose = np.array([0., 0., 0., 0., 0.005, 0., 0.9999])
        # base_pose = np.array([0., 0., 0., 0., 0, 0., 1])
        base_twist = np.array([0., 0., 0., 0., 0., 0.])


    robot.sense()
    q_init = robot.getPositionReference()
    q_init = robot.eigenToMap(q_init)

    print(colorama.Fore.CYAN + 'RobotInterface created!' + colorama.Fore.RESET)

elif talos_param:
    rospy.Subscriber("/inria_controller/pub_joint_read", JointState, joint_read_callback)
    rospy.Subscriber("/inria_controller/pub_joint_cmd", JointState, joint_cmd_callback)

    while joint_cmd is None or joint_read is None:
        print('Waiting for joint data')
        rospy.sleep(0.01)

    print(colorama.Fore.CYAN + 'TalosInterface created!' + colorama.Fore.RESET)

    q_init = {jname: jpos for jname, jpos in zip(joint_names, joint_cmd)}
    base_pose = np.array([0., 0., 0., 0., 0.005, 0., 0.9999])
    base_twist = [0, 0, 0, 0, 0, 0]


else:
    print(colorama.Fore.CYAN + 'RobotInterface not created' + colorama.Fore.RESET)
    base_pose = np.array([0.0, 0.0, 0.0, 0., 0.0, 0.0, 1.0])
    base_twist = np.zeros(6)
    robot = None

    q_init = {"leg_left_1_joint": -7.94464787047849e-13,
              "leg_left_2_joint": -3.619964318427297e-05,
              "leg_left_3_joint": -0.6898316711444555,
              "leg_left_4_joint": 1.3311371331572148,
              "leg_left_5_joint": -0.6413054620145101,
              "leg_left_6_joint": 3.619964990352649e-05,
              "leg_right_1_joint": -7.94464787047849e-13,
              "leg_right_2_joint": -3.619964318427297e-05,
              "leg_right_3_joint": -0.6898316711444555,
              "leg_right_4_joint": 1.3311371331572148,
              "leg_right_5_joint": -0.6413054620145101,
              "leg_right_6_joint": 3.619964990352649e-05,
              "torso_1_joint": 0.0002205888120137109,
              "torso_2_joint": 0.027120105655840155,
              "arm_left_1_joint": -0.03397840856227864,
              "arm_left_2_joint": 0.3086614029301087,
              "arm_left_3_joint": 0.03272967979826196,
              "arm_left_4_joint": -1.0322427349026453,
              "arm_left_5_joint": 0.004009140644162072,
              "arm_left_6_joint": 0.00358625686761845,
              "arm_left_7_joint": 0.02561515245675291,
              "arm_right_1_joint": -0.03397840856227864,
              "arm_right_2_joint": -0.3086614029301087,
              "arm_right_3_joint": 0.03272967979826196,
              "arm_right_4_joint": -1.0322427349026453,
              "arm_right_5_joint": 0.004009140644162072,
              "arm_right_6_joint": 0.00358625686761845,
              "arm_right_7_joint": 0.02561515245675291,
              "head_1_joint": -0.027120105662825997,
              "head_2_joint": -0.0002205888117432746}

rospy.sleep(2)

l_foot_link_name = 'left_sole_link'
r_foot_link_name = 'right_sole_link'
base_link_name = 'torso_1_link'

'''
fixed joints
'''
fixed_joint_map = dict()
fixed_joint_map.update({
    "arm_left_5_joint": 0.004009140644162072,
    "arm_left_6_joint": 0.00358625686761845,
    "arm_left_7_joint": 0.02561515245675291,
    "arm_right_5_joint": 0.004009140644162072,
    "arm_right_6_joint": 0.00358625686761845,
    "arm_right_7_joint": 0.02561515245675291,
    "head_1_joint": -0.027120105662825997,
    "head_2_joint": -0.0002205888117432746})

for fixed_joint in fixed_joint_map:
    if fixed_joint in q_init:
        del q_init[fixed_joint]

'''
Initialize Horizon problem
'''
ns = 31
T = 1.5
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joint_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 fixed_joint_map=fixed_joint_map)


bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)



ti = TaskInterface(prb=prb, model=model)

ti.setTaskFromYaml(rospkg.RosPack().get_path('talos_horizon') + '/config/talos_config.yaml')

qmin = np.array(model.kd.q_min())
qmax = np.array(model.kd.q_max())
prb.createResidual("q_limit_lower", 100 * utils.barrier(model.q - qmin))
prb.createResidual("q_limit_upper", 100 * utils.barrier1(model.q - qmax))

'''
adding minimization of angular momentum
'''
cd_fun = ti.model.kd.computeCentroidalDynamics()
h_lin, h_ang, dh_lin, dh_ang = cd_fun(model.q, model.v, model.a)
min_ang_mom_weight = prb.createParameter('min_ang_mom_weight', 1)
min_ang_mom_weight.assign(1e-1)
prb.createIntermediateResidual('min_angular_mom', min_ang_mom_weight * dh_ang)

'''
Foot vertices relative distance constraint
'''
pos_lf = model.kd.fk(l_foot_link_name)(q=model.q)['ee_pos']
pos_rf = model.kd.fk(r_foot_link_name)(q=model.q)['ee_pos']
base_ori = model.kd.fk(base_link_name)(q=model.q)['ee_rot']
rel_dist = base_ori.T @ (pos_lf - pos_rf)

prb.createResidual('relative_distance_lower_y', 100. * utils.barrier(rel_dist[1] - 0.25))

f0 = [0, 0, kin_dyn.mass() / 8 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)


# connects the frame (leg_left_6_link) with the task (foot_contact_l)
contact_task_dict = {'left_sole_link': 'foot_contact_l',
                     'right_sole_link': 'foot_contact_r'}

z_task_dict = {'left_sole_link': 'foot_z_l',
               'right_sole_link': 'foot_z_r'}

f_reg_dict = {'left_sole_link': ['f_left_regularization_0', 'f_left_regularization_1', 'f_left_regularization_2', 'f_left_regularization_3'],
              'right_sole_link': ['f_right_regularization_0', 'f_right_regularization_1', 'f_right_regularization_2', 'f_right_regularization_3']}

pm = pymanager.PhaseManager(ns + 1)
rs = pyrosserver.RosServerClass(pm)

pgm = PhaseGaitWrapper(ti, pm, model.getContactMap(), swing_task_list=list(z_task_dict.values()))

z_traj = np.zeros([7, 1])
z_traj[2] = 0.


for contact_name, timeline in pgm.getContactTimelines().items():
    pgm.getStancePhases()[contact_name].addItem(ti.getTask(contact_task_dict[contact_name]))
    # for contact_reg_task in f_reg_dict[contact_name]:
    #     pgm.getStancePhases()[contact_name].addItemReference(ti.getTask(contact_reg_task), np.array(f0))
    pgm.getFlightPhases()[contact_name].addItemReference(ti.getTask(z_task_dict[contact_name]), z_traj)

pgm.initializeTimeline()

model.q.setBounds(model.q0, model.q0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=0)
model.v.setBounds(model.v0, model.v0, nodes=ns)
model.q.setInitialGuess(ti.model.q0)

# finalize taskInterface and solve bootstrap problem
ti.finalize()

# pm.update()
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

if joystick_flag:
    from joy_commands import JoyCommands

    jc = JoyCommands()

gait_manager_ros = GaitManagerROS(pgm)
gait_manager_ros.setBasePoseWeight(0.5)
gait_manager_ros.setBaseRotWeight(0.1)

robot_joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]
q_robot = np.zeros(len(robot_joint_names))
qdot_robot = np.zeros(len(robot_joint_names))

rospy.set_param('/horizon/walk/step_duration', 15)
rospy.set_param('/horizon/walk/double_stance', 10)

while not rospy.is_shutdown():

    # set initial state and initial guess
    shift_num = -1

    x_opt = solution['x_opt']
    xig = np.roll(x_opt, shift_num, axis=1)
    for i in range(abs(shift_num)):
        xig[:, -1 - i] = x_opt[:, -1]
    prb.getState().setInitialGuess(xig)
    prb.setInitialState(x0=xig[:, 0])

    # if closed_loop:
    #     set_state_from_robot(robot_joint_names, q_robot, qdot_robot, fixed_joint_map)

    pm.shift()

    # receive msgs from joystick and publishes to ROS topic
    if joystick_flag:
        jc.run()

    # receive msgs from ros topic and send commands to robot
    gait_manager_ros.run()

    pm.update()

    ti.rti()
    solution = ti.solution

    fmap = {frame: solution[f'f_{frame}'][:, 0] for frame in model.fmap.keys()}
    tau = model.id_fn.call(solution['q'][:, 0], solution['v'][:, 0].tolist(), solution['a'][:, 0].tolist(), fmap)

    if talos_param:
        sol_msg = JointState()
        sol_msg.name = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]
        sol_msg.position = solution['q'][7:, 1].tolist()
        sol_msg.velocity = solution['v'][6:, 1].tolist()
        sol_msg.effort = tau.elements()[6:]
        solution_publisher.publish(sol_msg)
    elif xbot_param:
        sol_msg = WBTrajectory()
        sol_msg.header.frame_id = 'world'
        sol_msg.header.stamp = rospy.Time.now()

        sol_msg.joint_names = [elem for elem in kin_dyn.joint_names() if elem not in ['universe', 'reference']]

        sol_msg.q = solution['q'][:, 1].tolist()
        sol_msg.v = solution['v'][:, 1].tolist()
        sol_msg.a = solution['a'][:, 1].tolist()

        for frame in model.getForceMap():
            sol_msg.force_names.append(frame)
            sol_msg.f.append(
                Vector3(x=solution[f'f_{frame}'][0, 0], y=solution[f'f_{frame}'][1, 0], z=solution[f'f_{frame}'][2, 0]))

        sol_msg.tau = tau.elements()

        solution_publisher.publish(sol_msg)

    if robot is None:
        repl.frame_force_mapping = {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()}
        repl.publish_joints(solution['q'][:, 0])
        repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)

    rs.run()

    rate.sleep()






















