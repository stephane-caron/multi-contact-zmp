#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2015-2016 Stephane Caron <stephane.caron@normalesup.org>
#
# This file is part of multi-contact-zmp
# <https://github.com/stephane-caron/multi-contact-zmp>.
#
# multi-contact-zmp is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# multi-contact-zmp is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# multi-contact-zmp. If not, see <http://www.gnu.org/licenses/>.

import IPython
import Tkinter as tk
import contact_stability.srv
import geometry_msgs.msg
import numpy
import openravepy
import os
import rospy
import sys
import time
import tkFileDialog

try:
    import pymanoid
except:
    script_path = os.path.realpath(__file__)
    sys.path.append(os.path.dirname(script_path) + '/../pymanoid')
    import pymanoid

from numpy import array, cross, dot, sqrt, zeros
from scipy.spatial.qhull import QhullError
from threading import Lock, Thread

from editor.control import PendulumController, StationaryController
from editor.plan import MotionPlan
from editor.plots import PlotData

from pymanoid.exceptions import OptimalNotFound
from pymanoid.robots import JVRC1

try:
    from full_support_area import draw_full_support_area
except:
    script_path = os.path.realpath(__file__)
    sys.path.append(os.path.dirname(script_path) + '/../full_support_area')
    from full_support_area import draw_full_support_area


# Settings
after_step_duration = 1.0  # [s]
dt = 3e-2  # [s]
gravity = array([0.0, 0.0, -9.81])  # [m/s^2]
zmp_normal = array([0.0, 0.0, 1.0])
ctrl_timesteps = 100

# IK params
G_com = 1. / dt
G_dof = 0.005 / dt
G_link = 1. / dt
G_link_free = 0.03 / dt
G_ref = 0.005 / dt
K_doflim = 50.0  # [s^-1]
qd_lim = 0.5  # [rad/s]
w_cam = 000.2
w_com = 005.
w_dof = 000.1
w_link = 100.
w_link_free = 10.
w_ref = 000.1
w_reg = 001.

gui_defaults = {
    'show_forces': False,
    'show_targets': False,
}

# Global variables
CMU = ("CMU Sans Serif", 12)
area_comp_times = {1: [], 2: [], 3: []}
cam_id = 0  # identifier of current visualization camera
collision_handle = None  # collision handler for OpenRAVE
compute_com_area = None  # ROS service call
compute_support_area = None  # ROS service call
frame_index = 0
init_plan_handles = None
last_ctrl_update = None
motion_plan = None
plot_data = PlotData()
pushed_planning_segment = None
real_com_handles = []
real_com_traj = []
real_zmp_handles = []
real_zmp_traj = []
record_screen = False
robot_lock = None
save_next_area = False
save_trajectories = True
saved_areas = []
step_plan_times = []
step_t = 1000.
support_area_handles = {}
support_area_repr = {}
window_id = "root"
x_controller = None
y_controller = None

q_halfsit_jvrc1 = array([
    -0.38, -0.01, 0., 0.72, -0.01, -0.33, -0.38, 0.02, 0., 0.72, -0.02,
    -0.33, 0., 0.13, 0., 0., 0., 0., -0.052, -0.17, 0., -0.52, 0., 0., 0.,
    0., 0., 0., 0., 0., 0., -0.052, 0.17, 0., -0.52, 0., 0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0., 0., 0., 0., 0.])


def check_contact_forces(com, comdd, camd):
    if camd is None:  # not set yet
        camd = numpy.zeros(3)
    try:
        gravity = pymanoid.get_gravity()
        wrench = numpy.hstack([robot.mass * (comdd - gravity), camd])
        contacts = motion_plan.cur_stance.contacts
        return contacts.find_supporting_forces(wrench, com)
    except OptimalNotFound:
        print "No contact forces here (t=%.2f)." % step_t


class Pendulum(object):

    def __init__(self, robot, visible=True):
        self.arrow = None
        self.com = pymanoid.Cube(
            0.01, robot.com, name='Pendulum-COM', color='b')
        self.com.pd = zeros(3)
        self.zmp = pymanoid.Cube(
            0.015, self.com.pos + array([0, 0, 1]), name='Pendulum-ZMP',
            color='b', visible=True)
        self.is_visible = visible

    def draw(self):
        if self.is_visible:
            self.arrow = pymanoid.draw_arrow(
                pendulum.zmp.p, pendulum.com.p, linewidth=0.01,
                color=(0., 0., 1., 1.))

    def set_visible(self, visible):
        self.is_visible = visible
        if not self.is_visible:
            self.arrow = None
        self.com.set_visible(visible)
        self.zmp.set_visible(visible)


def free_alpha():
    global step_t
    alpha_i = 0.03
    alpha_f = 0.9
    try:
        x = step_t / motion_plan.cur_stance.step_duration
        # the min is important here, as step_t may be very large
        alpha = min(alpha_f, alpha_i + (alpha_f - alpha_i) * x ** 2)
    except:
        alpha = alpha_i
    # print "alpha =", alpha
    return alpha


def com_proj(pos):
    return array([pos[0], pos[1], motion_plan.com_height])


def convert_ros_response(res):
    vertices = [array([v.x, v.y, v.z]) for v in res.vertices]
    rays = [array([r.x, r.y, r.z]) for r in res.rays]
    return vertices, rays


def convert_contacts_to_ros(contacts):
    return [
        contact_stability.msg.Contact(
            geometry_msgs.msg.Point(
                contact.p[0],
                contact.p[1],
                contact.p[2]),
            geometry_msgs.msg.Quaternion(
                contact.pose[1],   # x
                contact.pose[2],   # y
                contact.pose[3],   # z
                contact.pose[0]),  # w comes first in OpenRAVE convention
            contact.X, contact.Y,
            contact.friction)
        for contact in contacts]


def compute_zmp_support_area_ros(contacts, p_in, z_out, name, color):
    req = contact_stability.srv.PendularAreaRequest(
        contacts=convert_contacts_to_ros(contacts),
        mass=robot.mass,
        p_in=geometry_msgs.msg.Point(p_in[0], p_in[1], p_in[2]),
        z_out=z_out)
    try:
        # This commented block was used to gather computation times reported in
        # Table III of the paper.
        #
        # if False:
        #     t0 = time.time()
        #     res = compute_support_area(req)
        # if False:
        #     G = contacts.compute_wrench_cone(p_in)
        #     t0 = time.time()
        #     f = numpy.array([0., 0., robot.mass * 9.81])
        #     tau = zeros(3)
        #     wrench = numpy.hstack([f, tau])
        #     check = all(dot(G, wrench) <= 0)
        # if False:
        #     t0 = time.time()
        #     contacts.find_static_supporting_forces(p_in, robot.mass)
        # area_comp_times[contacts.nb_contacts].append(time.time() - t0)
        res = compute_support_area(req)
        return convert_ros_response(res)
    except rospy.ServiceException as e:
        rospy.logwarn(str(e))
        return [], []


def print_area_comp_times():
    from numpy import average, std
    for i in [1, 2, 3]:
        print "$%.1f \\pm %.1f$ (%d) &" % (
            1000 * average(area_comp_times[i]),
            1000 * std(area_comp_times[i]),
            len(area_comp_times[i])),
    print ""


def compute_support_area_thread():
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if 'ZMP' in support_area_handles and not gui.show_zmp_support_area:
            delete_support_area_display('ZMP')
        if 'ZMP-full' in support_area_handles and not gui.show_zmp_support_area:
            delete_support_area_display('ZMP-full')
        if not motion_plan or not motion_plan.started \
                or not gui.show_zmp_support_area:
            rate.sleep()
            continue
        if x_controller or y_controller:
            # color = (0.5, 0., 0., 0.5)
            color = (0., 0.5, 0., 0.5)
            com_pos = pendulum.com.p
            z_out = pendulum.zmp.z
        else:
            color = (0., 0., 0.5, 0.5)
            com_pos = motion_plan.cur_stance.zmp.pos
            com_pos[2] = motion_plan.com_height
            z_out = motion_plan.cur_stance.zmp.z
            if gui.auto_advance:
                rate.sleep()
                continue
        zmp_vertices, zmp_rays = compute_zmp_support_area_ros(
            motion_plan.cur_stance.contacts, com_pos, z_out,
            name='ZMP', color=color)
        update_support_area_display('ZMP', zmp_vertices, zmp_rays, color)
        if motion_plan.cur_stance:  # may haved changed since beginning of loop
            support_area_handles['ZMP-full'] = draw_full_support_area(
                motion_plan.cur_stance.contacts, zmp_normal, z_out, alpha=0.1,
                draw_gen_polygons=False, color=(0.5, 0.5, 0.),
                plot_type=6, linewidth=2)
        rate.sleep()


def compute_static_stability_thread():
    rate = rospy.Rate(60)
    global kron_com_vertices
    while not rospy.is_shutdown():
        if 'COM-static' in support_area_handles \
                and not gui.show_com_support_area:
            delete_support_area_display('COM-static')
        if not motion_plan or not motion_plan.started \
                or 'COM-static' in support_area_handles \
                or not gui.show_com_support_area:
            rate.sleep()
            continue
        color = (0.1, 0.1, 0.1, 0.5)
        req = contact_stability.srv.StaticAreaRequest(
            contacts=convert_contacts_to_ros(motion_plan.cur_stance.contacts),
            mass=robot.mass, z_out=motion_plan.com_height)
        try:
            res = compute_com_area(req)
            vertices = [array([v.x, v.y, v.z]) for v in res.vertices]
            update_support_area_display('COM-static', vertices, [], color)
        except rospy.ServiceException:
            delete_support_area_display('COM-static')
        rate.sleep()


def update_support_area_display(name, vertices, rays, color, plot_type=6):
    global support_area_handles
    global support_area_repr
    global save_next_area
    try:
        support_area_handles[name] = pymanoid.draw_2d_cone(
            vertices, rays, zmp_normal, plot_type=plot_type, color=color)
        support_area_repr[name] = (vertices, rays)
        if save_next_area:
            saved_areas.append(pymanoid.draw_2d_cone(
                vertices, rays, zmp_normal, plot_type=2, color=color))
            save_next_area = False
    except IndexError:  # convex hull singularity
        pass
    except QhullError:  # also a convex hull singularity case
        pass


def delete_support_area_display(name):
    global support_area_handles
    if name in support_area_handles:
        del support_area_handles[name]


def update_support_areas():
    global support_area_handles
    support_area_handles = {}  # dedicated threads will recompute them


def customize_joint_limits(robot):
    robot.update_dof_limits(robot.L_SHOULDER_P, q_min=-1.5)
    robot.update_dof_limits(robot.R_SHOULDER_P, q_min=-1.5)
    robot.update_dof_limits(robot.L_SHOULDER_R, q_min=0.3)
    robot.update_dof_limits(robot.R_SHOULDER_P, q_max=0.)
    robot.update_dof_limits(robot.R_SHOULDER_R, q_max=-0.2)
    robot.scale_dof_limits(0.95)


def set_motion_active_dofs(robot):
    """Set active DOFs used while performing the motion."""
    active_dofs = []
    active_dofs += robot.chest_dofs
    active_dofs += robot.free_dofs
    active_dofs += robot.left_arm_dofs
    active_dofs += robot.left_leg_dofs
    active_dofs += robot.right_arm_dofs
    active_dofs += robot.right_leg_dofs
    robot.set_active_dofs(active_dofs)


def ik_add_contact_objectives(stance):
    if 'left_foot' in stance.contacts:
        contact = stance.contacts['left_foot']
        G, w = G_link, w_link
        robot.add_contact_objective(robot.left_foot, contact, G, w)
    elif 'left_foot' in stance.free:
        contact = stance.free['left_foot']
        G, w = G_link, w_link_free
        robot.add_contact_vargain_objective(
            robot.left_foot, contact, G, w, free_alpha)
    if 'right_foot' in stance.contacts:
        contact = stance.contacts['right_foot']
        G, w = G_link, w_link
        robot.add_contact_objective(robot.right_foot, contact, G, w)
    elif 'right_foot' in stance.free:
        contact = stance.free['right_foot']
        G, w = G_link, w_link_free
        robot.add_contact_vargain_objective(
            robot.right_foot, contact, G, w, free_alpha)
    if 'left_hand' in stance.contacts:
        contact = stance.contacts['left_hand']
        G, w = G_link, w_link
        robot.add_contact_objective(robot.left_hand, contact, G, w)
    elif 'left_hand' in stance.free:
        contact = stance.free['left_hand']
        G, w = G_link, w_link_free
        robot.add_contact_vargain_objective(
            robot.left_hand, contact, G, w, free_alpha)
    if 'right_hand' in stance.contacts:
        contact = stance.contacts['right_hand']
        G, w = G_link, w_link
        robot.add_contact_objective(robot.right_hand, contact, G, w)
    elif 'right_hand' in stance.free:
        contact = stance.free['right_hand']
        G, w = G_link, w_link_free
        robot.add_contact_vargain_objective(
            robot.right_hand, contact, G, w, free_alpha)


def generate_posture(stance, init=False):
    robot_lock.acquire(True)
    set_motion_active_dofs(robot)
    q_init = q_halfsit_jvrc1.copy()
    dof_objectives = [
        (robot.R_SHOULDER_R, -.5),
        (robot.WAIST_P, 0.),
        (robot.WAIST_Y, 0.),
        (robot.L_SHOULDER_R, +.5)]
    for (dof_id, dof_ref) in dof_objectives:
        robot.set_dof_values([dof_ref], [dof_id])
        q_init[dof_id] = dof_ref
    if init:
        robot.set_dof_values([-1], [robot.R_SHOULDER_P])
        robot.set_dof_values([-1], [robot.L_SHOULDER_P])
        robot.set_dof_values([0.8], dof_indices=[robot.TRANS_Z])
    target_com = com_proj(stance.zmp.pos)
    pendulum.com.set_pos(target_com)
    robot.init_ik(qd_lim=100., K_doflim=K_doflim)
    robot.add_com_objective(pendulum.com, G_com, w_com)
    robot.add_posture_objective(q_halfsit_jvrc1, G_ref, w_ref)
    for (dof_id, dof_ref) in dof_objectives:
        robot.add_dof_objective(dof_id, dof_ref, G_dof, w_dof)
    ik_add_contact_objectives(stance)
    robot.solve_ik(dt, conv_tol=1e-3, max_it=100, debug=False)
    robot_lock.release()
    update_gui_after_generate_posture()


def update_gui_after_generate_posture():
    if pendulum.zmp and pendulum.com and motion_plan.cur_stance:
        pendulum.zmp.set_pos(motion_plan.cur_stance.zmp.pos)
        pendulum.com.set_pos(com_proj(pendulum.zmp.pos))
    update_support_areas()


def init_motion_ik():
    robot_lock.acquire(True)
    set_motion_active_dofs(robot)
    dof_objectives = [(robot.R_SHOULDER_R, -.5), (robot.L_SHOULDER_R, +.5)]
    robot.init_ik(qd_lim, K_doflim)
    ik_add_contact_objectives(motion_plan.cur_stance)
    robot.add_com_objective(pendulum.com, G_com, w_com)
    robot.add_constant_cam_objective(w_cam)
    robot.add_posture_objective(q_halfsit_jvrc1, G_ref, w_ref)
    for (dof_id, dof_ref) in dof_objectives:
        robot.add_dof_objective(dof_id, dof_ref, G_dof, w_dof)
    robot.add_velocity_regularization(w_reg)
    robot_lock.release()


class MainWindow(tk.Frame):

    def __init__(self, master=None, grid_size=(3, 5)):
        tk.Frame.__init__(self, master)
        master.rowconfigure(0, weight=1)
        master.columnconfigure(0, weight=1)
        for row in xrange(grid_size[0]):
            tk.Grid.rowconfigure(self, row, weight=1)
        for col in xrange(grid_size[1]):
            tk.Grid.columnconfigure(self, col, weight=1)
        self.grid(sticky='news')
        self.grid_size = grid_size
        self.init_gui_variables()
        self.init_variables()
        self.add_buttons()

    def init_gui_variables(self):
        self._show_com_support_area = tk.IntVar()
        self._show_forces = tk.IntVar()
        self._show_targets = tk.IntVar()
        self._show_zmp_support_area = tk.IntVar()
        self.init_plan_handles = []

    def init_variables(self):
        self.auto_advance = False

    @property
    def show_com_support_area(self):
        return self._show_com_support_area.get() == 1

    @property
    def show_forces(self):
        return self._show_forces.get() == 1

    @property
    def show_targets(self):
        return self._show_targets.get() == 1

    @property
    def show_zmp_support_area(self):
        return self._show_zmp_support_area.get() == 1

    def add_buttons(self):
        row = 0
        self.open_plan_btn = tk.Button(
            self, text="Open", command=self.open_plan, font=CMU)
        # self.save_btn = tk.Button(
        # self, text="Save", command=self.save, font=CMU, state=tk.DISABLED)
        self.save_as_btn = tk.Button(
            self, text="Save as...", command=self.save_as, font=CMU)
        self.restart_btn = tk.Button(
            self, text="Restart", command=self.restart, font=CMU)
        self.robot_transp_btn = tk.Button(
            self, text="Robot transp.", command=self.switch_robot_transp,
            font=CMU)
        # self.save_area_btn = tk.Button(
        # self, text="Save area", command=self.save_area, font=CMU)
        self.quit_btn = tk.Button(
            self, text="Quit", command=root.quit, font=CMU)
        self.open_plan_btn.grid(row=row, column=0, sticky='news')
        # self.save_btn.grid(row=row, column=1, sticky='news')
        self.save_as_btn.grid(row=row, column=1, sticky='news')
        self.restart_btn.grid(row=row, column=2, sticky='news')
        self.robot_transp_btn.grid(row=row, column=3, sticky='news')
        # self.save_area_btn.grid(row=row, column=4, sticky='news')
        self.quit_btn.grid(row=row, column=4, sticky='news')

        row = 1
        play_text = "Play & Record" if record_screen else "Play"
        self.prev_button = tk.Button(
            self, text="<", command=self.click_prev_button, font=CMU)
        self.stance_label = tk.StringVar()
        self.stance_label_ = tk.Label(
            self, textvariable=self.stance_label, font=CMU)
        self.stance_label.set("Motion plan is empty")
        self.next_button = tk.Button(
            self, text=">", command=self.click_next_button, font=CMU)
        self.step_button = tk.Button(
            self, text="Step", command=self.click_step_button, font=CMU)
        self.play_cb = tk.Checkbutton(
            self, text=play_text, command=self.toggle_play, font=CMU)
        self.prev_button.grid(row=row, column=0, sticky='news')
        self.stance_label_.grid(row=row, column=1, sticky='news')
        self.next_button.grid(row=row, column=2, sticky='news')
        self.step_button.grid(row=row, column=3, sticky='news')
        self.play_cb.grid(row=row, column=4, sticky='news')

        row = 2
        self.show_plan_toggled_ = tk.IntVar()
        self.show_plan = tk.Checkbutton(
            self, text="Show plan", command=self.toggle_show_plan,
            var=self.show_plan_toggled_, font=CMU)
        self.show_plan.select()
        self.com_sa_cb = tk.Checkbutton(
            self, text="Show COM polygon", variable=self._show_com_support_area,
            font=CMU)
        self.zmp_sa_cb = tk.Checkbutton(
            self, text="Show ZMP areas", variable=self._show_zmp_support_area,
            font=CMU)
        self.zmp_sa_cb.select()
        self.targets_cb = tk.Checkbutton(
            self, text="Show targets", variable=self._show_targets,
            command=self.toggle_targets, font=CMU)
        if gui_defaults['show_targets']:
            self.targets_cb.select()
        self.forces_cb = tk.Checkbutton(
            self, text="Show forces", variable=self._show_forces,
            command=self.toggle_forces, font=CMU)
        if gui_defaults['show_forces']:
            self.forces_cb.select()
        self.show_plan.grid(row=row, column=0, sticky='news')
        self.com_sa_cb.grid(row=row, column=1, sticky='news')
        self.zmp_sa_cb.grid(row=row, column=2, sticky='news')
        self.targets_cb.grid(row=row, column=3, sticky='news')
        self.forces_cb.grid(row=row, column=4, sticky='news')

    def open_plan(self):
        global motion_plan
        if motion_plan.path:
            initialdir, initialfile = os.path.split(motion_plan.path)
            path = tkFileDialog.askopenfilename(
                parent=self.master, initialfile=initialfile,
                initialdir=initialdir)
        else:
            path = tkFileDialog.askopenfilename(parent=self.master)
        if path:
            load_motion_plan(path)

    def save(self):
        return self.save_as(motion_plan.path)

    def save_as(self, path=None):
        if path:
            return motion_plan.dump(path)
        if motion_plan.path:
            initialdir, initialfile = os.path.split(motion_plan.path)
            path = tkFileDialog.asksaveasfilename(
                parent=self.master, initialfile=initialfile,
                initialdir=initialdir)
        else:
            path = tkFileDialog.asksaveasfilename(parent=self.master)
        if path:
            motion_plan.dump(path)

    def restart(self):
        load_motion_plan(None)

    def click_step_button(self):
        if x_controller:  # we are executing step
            plan_next_step()
        else:
            start_step()

    def click_prev_button(self):
        global x_controller
        global y_controller
        motion_plan.step_back()
        generate_posture(motion_plan.cur_stance)
        x_controller = StationaryController(pendulum.zmp.x)
        y_controller = StationaryController(pendulum.zmp.y)
        self.step_button.configure(text="Plan next step", state=tk.NORMAL)
        self.update_labels()

    def click_next_button(self):
        global x_controller
        global y_controller
        motion_plan.step()
        generate_posture(motion_plan.cur_stance)
        x_controller = StationaryController(pendulum.zmp.x)
        y_controller = StationaryController(pendulum.zmp.y)
        self.step_button.configure(text="Plan next step", state=tk.NORMAL)
        self.update_labels()

    def toggle_targets(self):
        motion_plan.toggle_targets()
        com_ghost.set_visible(not com_ghost.is_visible)

    def toggle_forces(self):
        if not self.show_forces:
            self.force_handles = []

    @property
    def show_plan_toggled(self):
        return self.show_plan_toggled_.get() > 0

    def toggle_show_plan(self):
        if not self.show_plan_toggled:
            self.init_plan_handles = []
            # self.cur_segment_handle = None
            return
        if motion_plan.nb_stances < 2:
            return
        path = [stance.zmp.pos for stance in motion_plan.stances]
        color1 = (0.5, 0.3, 0.)
        env = pymanoid.get_env()
        self.init_plan_handles = [
            env.drawlinelist(array(path), linewidth=2, colors=color1),
            env.drawlinelist(array(path[1:]), linewidth=2, colors=color1)]
        # com_path = [com_proj(p) for p in path]
        # self.init_plan_handles.extend([
        #    env.drawlinelist(array(com_path), linewidth=2, colors=color1),
        #    env.drawlinelist(array(com_path[1:]), linewidth=2, colors=color1)])

    def toggle_play(self):
        if self.auto_advance:
            self.auto_advance = False
        else:
            self.step_button.configure(state=tk.DISABLED)
            self.auto_advance = True
        if not x_controller or not y_controller:
            start_step()

    def update_display(self):
        global camd
        global pushed_planning_segment
        pendulum.draw()
        # env = pymanoid.get_env()
        if pushed_planning_segment is not None:
            if self.show_plan_toggled:
                p0, p1 = pushed_planning_segment
                # self.cur_segment_handle = env.drawlinelist(
                # array([p0, p1]), linewidth=3, colors=(0., 0., 0.5, 1.))
            pushed_planning_segment = None
        if self.show_forces and camd is not None:
            next_handles = []
            F = check_contact_forces(pendulum.com.p, pendulum.com.pdd, camd)
            next_handles = [pymanoid.draw_force(c, f) for (c, f) in F]
            self.force_handles = next_handles

    def update_labels(self):
        self.stance_label.set(
            "Stance %d\nStep duration %.2f s" % (
                motion_plan.progress,
                motion_plan.cur_stance.step_duration))

    def switch_robot_transp(self):
        if robot.transparency > 0.6:
            robot.set_transparency(0.5)
            robot.set_visible(True)
        elif robot.transparency > 0.1:
            robot.set_transparency(0.)
            robot.set_visible(True)
        else:
            robot.set_transparency(1.)
            # avoids selection when clicking on invisible robot
            robot.set_visible(False)

    def save_area(self):
        global save_next_area
        save_next_area = True


def init_ros():
    global compute_com_area
    global compute_support_area
    rospy.init_node('real_time_control')
    rospy.wait_for_service(
        '/contact_stability/static/compute_support_area')
    rospy.wait_for_service(
        '/contact_stability/pendular/compute_support_area')
    compute_com_area = rospy.ServiceProxy(
        '/contact_stability/static/compute_support_area',
        contact_stability.srv.StaticArea)
    compute_support_area = rospy.ServiceProxy(
        '/contact_stability/pendular/compute_support_area',
        contact_stability.srv.PendularArea)


def collision_callback(report, physics):
    return openravepy.CollisionAction.Ignore


def compute_zmp(robot, q, qd, qdd, d_Z=0.):
    p_O, n = zeros(3), array([0, 0, 1])
    f_gi, tau_gi = robot.compute_gravito_inertial_wrench(q, qd, qdd, p_O)
    return p_O + (cross(n, tau_gi) + d_Z * f_gi) * 1. / dot(n, f_gi)


def execute_main_thread():
    rate = rospy.Rate(1. / dt)
    last_plot_time = time.time()
    last_plot_com = None
    last_plot_zmp = None
    plot_dt = 1e-1
    qdd = zeros(robot.nb_dofs)
    while not rospy.is_shutdown():
        if not motion_plan.started:
            rate.sleep()
            continue
        if x_controller and y_controller:
            step_controllers()
        zmp_target = motion_plan.cur_stance.zmp
        com_ghost.set_pos([zmp_target.x, zmp_target.y, motion_plan.com_height])
        omega2 = dot(zmp_normal, gravity) / dot(zmp_normal,
                                                pendulum.com.p - pendulum.zmp.p)
        pendulum.com.pdd = gravity - omega2 * (pendulum.com.p - pendulum.zmp.p)
        if omega2 > 0:  # we add a damping term between the ZMP and COM traj.
            pendulum.com.pdd -= 2 * sqrt(omega2) * pendulum.com.pd
        pendulum.com.set_pos(pendulum.com.p + dt * pendulum.com.pd +
                             .5 * dt * dt * pendulum.com.pdd)
        pendulum.com.pd = pendulum.com.pd + dt * pendulum.com.pdd
        qd_prev = robot.qd_full
        robot_lock.acquire(True)
        robot.step_ik(dt)
        robot_lock.release()
        # joint accelerations are estimated via a first-order filter on finite
        # differences of joint velocities
        qdd += .1 * ((robot.qd_full - qd_prev) / dt - qdd)
        robot_lock.acquire(True)  # kron
        new_zmp = compute_zmp(robot, robot.q, robot.qd, qdd, d_Z=pendulum.zmp.z)
        robot_lock.release()  # kron
        if save_trajectories and x_controller and y_controller \
                and time.time() - last_plot_time > plot_dt:
            if last_plot_com is None:
                last_plot_com = com_real.pos
            if last_plot_zmp is None:
                last_plot_zmp = zmp_real.pos
            real_com_handles.append(
                pymanoid.draw_line(last_plot_com, robot.com, linewidth=5,
                                   color='g'))
            real_zmp_handles.append(
                pymanoid.draw_line(last_plot_zmp, new_zmp, linewidth=3,
                                   color='r'))
            last_plot_com = com_real.pos
            last_plot_zmp = zmp_real.pos
            last_plot_time = time.time()
        if gui.show_forces:
            global camd
            camd = robot.compute_cam_rate(robot.q_full, robot.qd_full, qdd)
        com_real.set_pos(robot.com)
        zmp_real.set_pos(new_zmp)
        gui.update_display()
        if gui.auto_advance:
            plot_data.append(
                pendulum.zmp.p, pendulum.com.p, zmp_real.p, com_real.p)
            if record_screen and x_controller:
                take_screenshot()
        rate.sleep()


def take_screenshot():
    global frame_index
    fname = './recording/camera%d/%05d.png' % (cam_id, frame_index)
    os.system('import -window %s %s' % (window_id, fname))
    frame_index += 1


def step_controllers():
    global step_t
    step_duration = motion_plan.cur_step_duration
    if step_t < step_duration:
        pendulum.zmp.set_x(x_controller.control(step_t))
        pendulum.zmp.set_y(y_controller.control(step_t))
    elif step_t > step_duration + after_step_duration:
        if gui.auto_advance:
            plan_next_step()
        else:
            gui.step_button.configure(state=tk.NORMAL)
    step_t += dt


def plan_next_step():
    global x_controller
    global y_controller
    x_controller = None
    y_controller = None
    if 'ZMP' in support_area_repr:
        plot_data.append_support_area(support_area_repr['ZMP'])
    plan_is_over = motion_plan.step()
    if not plan_is_over:
        return
    update_support_areas()
    gui.step_button.configure(text="Perform step")
    gui.update_labels()
    if gui.auto_advance:
        start_step()


def start_step():
    global pushed_planning_segment
    global step_t
    global x_controller
    global y_controller
    if not motion_plan.started:
        return
    init_motion_ik()
    segment_beg = motion_plan.prev_stance.zmp.pos
    segment_end = motion_plan.cur_stance.zmp.pos
    gui.step_button.configure(text="Plan next step", state=tk.DISABLED)
    step_t = 0.
    omega2 = dot(zmp_normal, gravity) / dot(zmp_normal,
                                            pendulum.com.p - pendulum.zmp.p)
    step_duration = motion_plan.cur_stance.step_duration
    if abs(segment_end[0] - pendulum.zmp.pos[0]) > 1e-2:
        try:
            t0 = time.time()
            x_controller = PendulumController(
                pendulum.zmp.p[0], 0, segment_end[0], 0,
                omega2, step_duration, ctrl_timesteps)
            step_plan_times.append(time.time() - t0)
        except OptimalNotFound:
            print "Error in x-controller update."
    else:  # avoids singular KKT matrices
        x_controller = StationaryController(pendulum.zmp.p[0])
    if abs(segment_end[1] - pendulum.zmp.pos[1]) > 1e-2:
        try:
            t0 = time.time()
            y_controller = PendulumController(
                pendulum.zmp.p[1], 0, segment_end[1], 0,
                omega2, step_duration, ctrl_timesteps)
            step_plan_times.append(time.time() - t0)
        except OptimalNotFound:
            print "Error in y-controller update."
    else:  # avoids singular KKT matrices
        y_controller = StationaryController(pendulum.zmp.p[1])
    pushed_planning_segment = (segment_beg, segment_end)


def load_motion_plan(path=None):
    """If path is None, resets without reloading from file."""
    if motion_plan.started:
        motion_plan.stop()
        time.sleep(5 * dt)  # wait for main thread (a lock would be better)
    if path:
        motion_plan.reload(path)
    pendulum.zmp.set_pos(motion_plan.init_stance.zmp.pos)
    pendulum.com.set_pos(com_proj(pendulum.zmp.pos))
    generate_posture(motion_plan.init_stance, init=True)
    motion_plan.start()
    plan_next_step()
    # gui.save_btn.configure(state=tk.NORMAL)
    if gui.show_plan_toggled:
        gui.toggle_show_plan()
        gui.toggle_show_plan()


def plot_trajectories(area_indices=[1, 4, 6, 16, 17]):
    plot_data.plot_trajectories()
    plot_data.plot_support_areas(area_indices)


def launch_ipython():
    print ""
    print "==================================================================="
    print ""
    print "                   Welcome to the motion editor!                   "
    print ""
    print "To quit, click on the 'Quit' button in the GUI."
    print ""
    print "From this shell, available function calls include:"
    print ""
    print "    plot_trajectories()"
    print ""
    print "==================================================================="
    IPython.embed()
    root.quit()
    time.sleep(0.2)
    os._exit(0)  # dirty but avoids hangs from IPython's atexit callback


def stop_traj_recording():
    global save_trajectories
    save_trajectories = False


def clear_trajectories():
    global real_com_handles
    global real_zmp_handles
    real_com_handles = []
    real_zmp_handles = []


def show_saved_area(index, show):
    saved_areas[index][0].SetShow(show)


def blink_saved_area(index):
    show_saved_area(index, False)
    time.sleep(0.25)
    show_saved_area(index, True)


def set_camera_0():
    global cam_id
    pymanoid.get_viewer().SetCamera([
        [5.62570981e-01, -4.66264164e-01, 6.82723678e-01, -1.87612975e+00],
        [-8.26747455e-01, -3.18865212e-01, 4.63479905e-01, -1.98973417e+00],
        [1.59276025e-03, -8.25180408e-01, -5.64867026e-01, 3.30956650e+00],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    cam_id = 0


def set_camera_1():
    global cam_id
    pymanoid.get_viewer().SetCamera([
        [0.79144370,  0.24411156, -0.56038061,  3.21349812],
        [0.61108966, -0.29552771,  0.73432472, -2.30738068],
        [0.01364915, -0.92361947, -0.38306759,  2.12724948],
        [0.,  0.,  0.,  1.]])
    cam_id = 1


def set_camera_2():
    global cam_id
    pymanoid.get_viewer().SetCamera([
        [0., -1., 0., 0.7],
        [-1., 0., 0., 0.3],
        [0., 0., -1., 4.65],
        [0.,  0.,  0.,  1.]])
    cam_id = 2


if __name__ == '__main__':
    init_ros()
    pymanoid.init('editor/jvrc1_env.xml')
    env = pymanoid.get_env()
    collision_handle = env.RegisterCollisionCallback(collision_callback)
    viewer = env.GetViewer()
    motion_plan = MotionPlan(show_targets=gui_defaults['show_targets'])
    robot = JVRC1()
    robot.set_transparency(0.)
    customize_joint_limits(robot)

    pendulum = Pendulum(robot)
    com_ghost = pymanoid.Cube(
        0.01, robot.com, color='g', visible=motion_plan.show_targets)
    com_real = pymanoid.Cube(
        0.01, robot.com, color='r', visible=True)
    zmp_real = pymanoid.Cube(
        0.005, pendulum.zmp.pos, color='r', visible=True)

    if '-c1' in sys.argv:
        set_camera_1()
    elif '-c2' in sys.argv:
        set_camera_2()
    else:  # default camera
        set_camera_0()

    if '-r' in sys.argv or '--record' in sys.argv:
        from re import search
        record_screen = True
        print "Preparing for screen recording..."
        print "Please click on the OpenRAVE window."
        line = os.popen('/usr/bin/xwininfo | grep "Window id:"').readlines()[0]
        window_id = "0x%s" % search('0x([0-9a-f]+)', line).group(1)
        print "Window id:", window_id

    root = tk.Tk()
    root.title("Simulation manager")
    root.resizable(True, True)
    gui = MainWindow(root)

    robot_lock = Lock()

    if len(sys.argv) > 1 and sys.argv[1].endswith('.json'):
        load_motion_plan(sys.argv[1])

    main_thread = Thread(target=execute_main_thread, args=())
    main_thread.daemon = True
    main_thread.start()

    zmp_area_thread = Thread(target=compute_support_area_thread, args=())
    zmp_area_thread.daemon = True
    zmp_area_thread.start()

    com_poly_thread = Thread(target=compute_static_stability_thread, args=())
    com_poly_thread.daemon = True
    com_poly_thread.start()

    ipython_thread = Thread(target=launch_ipython, args=())
    ipython_thread.daemon = True
    ipython_thread.start()

    root.mainloop()
