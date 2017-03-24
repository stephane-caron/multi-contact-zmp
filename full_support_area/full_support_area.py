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

import os
import sys
import time

try:
    import pymanoid
except:
    script_path = os.path.realpath(__file__)
    sys.path.append(os.path.dirname(script_path) + '/../pymanoid')
    import pymanoid

from numpy import array, cross, dot, average, std, random
from scipy.spatial import ConvexHull
from threading import Lock, Thread


# Global variables
frame_index = 0
record_screen = False
screenshot_lock = Lock()


def compute_full_support_area(contacts, n, d, handles=None):
    """
    Compute the full ZMP support area in a virtual plane.

    The plane is defined by the equation

        dot(p, n) = d

    contacts -- any iterable set of pymanoid.Contact objects
    n -- plane normal, a numpy.array with shape (3,)
    d -- plane coordinate, a real value
    handles -- (optional) if not None, the OpenRAVE handles of drawn lines are
               appended to this list
    """
    O = d * n
    pos_poly, neg_poly = [], []
    for contact in contacts:
        for f in contact.force_span:
            for c in contact.vertices:
                p = O + cross(n, cross(c - O, f)) / dot(n, f)
                if dot(n, f) >= 0:
                    pos_poly.append(p)
                else:
                    neg_poly.append(p)
                if type(handles) is list:
                    handles.append(pymanoid.draw_line(
                        c, p, linewidth=1, color=(0.3, 0.3, 0.3)))
    return pos_poly, neg_poly


def compute_polygon_shadow(n, light_poly, ground_poly):
    """Polygons are described by their convex hulls.

    n -- normal vector used in ZMP computations
    light_poly -- polygon acting as light source
    ground_poly -- polygon whose shadow is projected under the light one

    """
    t = [n[2] - n[1], n[0] - n[2], n[1] - n[0]]
    n = array(n) / pymanoid.toolbox.norm(n)
    t = array(t) / pymanoid.toolbox.norm(t)
    b = cross(n, t)

    light_proj = [array([dot(t, p), dot(b, p)]) for p in light_poly]
    light_hull = ConvexHull(light_proj)
    ground_proj = [array([dot(t, p), dot(b, p)]) for p in ground_poly]
    ground_hull = ConvexHull(ground_proj)
    vertex2poly = {i: j for (i, j) in enumerate(ground_hull.vertices)}
    light_vertices = [light_proj[i] for i in light_hull.vertices]
    ground_vertices = [ground_proj[i] for i in ground_hull.vertices]
    mink_diff = [gv - lv for gv in ground_vertices for lv in light_vertices]

    try:
        u_low, u_high = pymanoid.draw.pick_2d_extreme_rays(mink_diff)
    except pymanoid.exceptions.UnboundedPolyhedron:
        big_dist = 1000  # [m]
        vertices = [
            array([-big_dist, -big_dist, light_poly[0][2]]),
            array([-big_dist, +big_dist, light_poly[0][2]]),
            array([+big_dist, -big_dist, light_poly[0][2]]),
            array([+big_dist, +big_dist, light_poly[0][2]])]
        return vertices, []

    nb_vertices = len(ground_vertices)
    vertex_indices = range(len(ground_vertices))

    def f_low(i):
        return cross(u_low, ground_vertices[i])

    def f_high(i):
        return cross(u_high, ground_vertices[i])

    v_low = min(vertex_indices, key=f_low)
    v_high = max(vertex_indices, key=f_high)
    vertices = [ground_poly[vertex2poly[vertex_index % nb_vertices]]
                for vertex_index in xrange(v_high, (v_low + nb_vertices + 1))]
    rays = [u_low[0] * t + u_low[1] * b,
            u_high[0] * t + u_high[1] * b]
    return vertices, rays


def draw_full_support_area(contacts, n, d, alpha=0.5, plot_type=7,
                           draw_lines=False, draw_gen_polygons=True,
                           color=(0., 0.5, 0.), linewidth=1.):
    """
    Draw the full ZMP support area in a virtual plane.

    The plane is defined by the equation

        dot(p, n) = d

    contacts -- any iterable set of pymanoid.Contact objects
    n -- plane normal, a numpy.array with shape (3,)
    d -- plane coordinate, a real value
    alpha -- transparency of output surfaces
    plot_type -- bitmask with 1 for vertices, 2 for edges and 4 for surface
    """
    handles = []
    rgba = list(color) + [alpha]
    # total_time = 0.
    # t0 = time.time()
    if draw_lines:
        pos_poly, neg_poly = compute_full_support_area(contacts, n, d, handles)
    else:
        pos_poly, neg_poly = compute_full_support_area(contacts, n, d)
    # total_time += time.time() - t0
    if pos_poly and not neg_poly:
        handles.append(pymanoid.draw_polygon(
            pos_poly, n, plot_type=plot_type, color=rgba, linewidth=linewidth))
        return handles  # , total_time
    elif neg_poly and not pos_poly:
        handles.append(pymanoid.draw_polygon(
            neg_poly, n, plot_type=plot_type, color=rgba, linewidth=linewidth))
        return handles, total_time
    # t0 = time.time()
    pos_v, pos_r = compute_polygon_shadow(n, neg_poly, pos_poly)
    neg_v, neg_r = compute_polygon_shadow(n, pos_poly, neg_poly)
    # total_time += time.time() - t0
    if draw_gen_polygons:
        handles.append(pymanoid.draw_polygon(
            pos_poly, n, plot_type=plot_type, color=(0.5, 0., 0.5, alpha),
            linewidth=linewidth))
        handles.append(pymanoid.draw_polygon(
            neg_poly, n, plot_type=plot_type, color=(0.5, 0., 0.5, alpha),
            linewidth=linewidth))
    handles.append(pymanoid.draw_2d_cone(
        pos_v, pos_r, n, plot_type=plot_type, color=rgba))
    handles.append(pymanoid.draw_2d_cone(
        neg_v, neg_r, n, plot_type=plot_type, color=rgba))
    return handles  # , total_time


def benchmark():
    times = []
    for _ in xrange(100000):
        for contact in contacts:
            contact.set_pos(random.random(3))
            contact.set_rpy(random.random(3))
        t0 = time.time()
        compute_full_support_area(
            contacts, plane_origin.T[:3, 2], plane_origin.z, None)
        times.append(time.time() - t0)
    print "%.2f +/- %.2f ms (%d)" % (
        1000 * average(times), 1000 * std(times), len(times))


def start_recording():
    global record_screen
    record_screen = True


def take_screenshot():
    global frame_index
    with screenshot_lock:
        fname = './recording/full_area/%05d.png' % frame_index
        os.system('import -window %s %s' % (window_id, fname))
        frame_index += 1


def rec_wait(duration=5., framerate=33):
    global frame_index
    global record_screen
    with screenshot_lock:
        fname = './recording/full_area/%05d.png' % frame_index
        os.system('import -window %s %s' % (window_id, fname))
        frame_index += 1
        for _ in xrange(int(framerate * duration)):
            next_fname = './recording/full_area/%05d.png' % frame_index
            os.system('cp %s %s' % (fname, next_fname))
            frame_index += 1


if __name__ == '__main__':
    if '--benchmark' not in sys.argv:
        print "\nUsage: %s [-c[0-3]] [--benchmark] [--record]\n" % sys.argv[0]
    pymanoid.init()
    # viewer.SetBkgndColor([.7, .7, .9])
    pymanoid.get_viewer().SetCamera(array(
        [[0.43294998, -0.40765653, 0.80397168, -1.7904253],
         [-0.90127634, -0.17995773, 0.39410173, -0.99202776],
         [-0.01597723, -0.89522699, -0.44532388, 1.42718434],
         [0., 0., 0., 1.]]))

    plane_origin = pymanoid.Box(
        X=0.1,
        Y=0.1,
        Z=0.1,
        pos=[-0.5, -1.5, 0.],
        color='b')

    if '-c0' in sys.argv:
        contacts = pymanoid.ContactSet({
            'C1': pymanoid.Contact(
                X=0.01,
                Y=0.01,
                Z=0.01,
                pos=array([0., 0., 0.]),
                rpy=array([0., 0., 0.]),
                friction=0.5,
                color='r',
                visible=True),
            'C2': pymanoid.Contact(
                X=0.01,
                Y=0.01,
                Z=0.01,
                pos=array([0., 0., 1.]),
                rpy=array([0., 3.14, 0.]),
                friction=0.5,
                color='r',
                visible=True)
        })
    elif '-c1' in sys.argv:
        contacts = pymanoid.ContactSet({
            'left_foot': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0., +0.25, 0.]),
                rpy=array([0., 0., 0.]),
                friction=0.5,
                color='r',
                visible=True),
            'right_foot': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0., -0.25, 0.]),
                rpy=array([0., 0., 0.]),
                friction=0.5,
                color='r',
                visible=True),
            'left_hand': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0.5, -0.25, 1.0]),
                rpy=array([3.14 / 2, 0., -3.14 / 2]),
                friction=0.5,
                color='r',
                visible=True)
        })
    elif '-c2' in sys.argv:
        contacts = pymanoid.ContactSet({
            'left_foot': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0., +0.25, 0.]),
                rpy=-array([0., 0., 0.]),
                friction=0.5,
                color='r',
                visible=True),
            'left_hand': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0., 1., 0.7]),
                rpy=-array([0.9, 0., 0.]),
                friction=0.5,
                color='r',
                visible=True)
        })
    else:
        contacts = pymanoid.ContactSet({
            'left_foot': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0., +0.25, 0.]),
                rpy=-array([0., 0., 0.]),
                friction=0.5,
                color='r',
                name='left_foot',
                visible=True),
            'right_foot': pymanoid.Contact(
                X=0.11,
                Y=0.07,
                pos=array([0., -0.25, 0.]),
                rpy=-array([0., 0., 0.]),
                friction=0.5,
                color='r',
                name='right_foot',
                visible=True)
        })

    if '--benchmark' in sys.argv:
        benchmark()

    if '--record' in sys.argv:
        from re import search
        print "Preparing for screen recording..."
        print "Please click on the OpenRAVE window."
        line = os.popen('/usr/bin/xwininfo | grep "Window id:"').readlines()[0]
        window_id = "0x%s" % search('0x([0-9a-f]+)', line).group(1)
        print "Window id:", window_id

    handles = []

    def draw_area_thread():
        global handles
        while True:
            new_handles = draw_full_support_area(
                contacts, plane_origin.T[:3, 2], plane_origin.z,
                draw_lines=False)
            handles = new_handles
            if record_screen:
                take_screenshot()
            else:
                time.sleep(1e-3)

    thread = Thread(target=draw_area_thread, args=())
    thread.daemon = True
    thread.start()

    import IPython
    IPython.embed()
