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

from numpy import array, cross, dot
from scipy.spatial import ConvexHull
from threading import Thread


def compute_polyhedron_shadow(n, light_poly, ground_poly, inf_dist=1000):
    light_hull = ConvexHull(light_poly)
    ground_hull = ConvexHull(ground_poly)
    light_vertices = [light_poly[i] for i in light_hull.vertices]
    ground_vertices = [ground_poly[i] for i in ground_hull.vertices]
    rays = [gv - lv for gv in ground_vertices for lv in light_vertices]
    return ground_vertices + [v + inf_dist * r
                              for v in ground_vertices for r in rays]


def compute_nmp_base_volume(contacts, plane):
    O, n = plane.p, plane.n
    pos_poly, neg_poly = [], []
    for contact in contacts:
        for f in contact.force_span:
            for c in contact.vertices:
                tau = cross(c - O, f)
                p = O + cross(n, tau) / dot(n, f) + dot(n, tau) * n
                if dot(n, f) >= 0:
                    pos_poly.append(p)
                else:
                    neg_poly.append(p)
    return pos_poly, neg_poly


def draw_nmp_support_volume(contacts, plane, alpha=0.5, plot_type=6):
    pos_poly, neg_poly = compute_nmp_base_volume(contacts, plane)
    if pos_poly and not neg_poly:
        return [
            pymanoid.draw_polyhedron(
                pos_poly, plot_type=plot_type, color=(0., 0.5, 0., alpha))]
    elif neg_poly and not pos_poly:
        return [
            pymanoid.draw_polyhedron(
                neg_poly, plot_type=plot_type, color=(0., 0.5, 0., alpha))]

    poly_color = (0.5, 0., 0., alpha)
    cone_color = (0., 0.5, 0., alpha)
    pos_cone = compute_polyhedron_shadow(plane.n, neg_poly, pos_poly)
    neg_cone = compute_polyhedron_shadow(plane.n, pos_poly, neg_poly)
    return [
        pymanoid.draw_polyhedron(
            pos_poly, plot_type=plot_type, color=poly_color),
        pymanoid.draw_polyhedron(
            neg_poly, plot_type=plot_type, color=poly_color),
        pymanoid.draw_polyhedron(
            pos_cone, plot_type=plot_type, color=cone_color),
        pymanoid.draw_polyhedron(
            neg_cone, plot_type=plot_type, color=cone_color)]


def draw_area_thread():
    handles = []
    while True:
        new_handles = draw_nmp_support_volume(contacts, plane)
        handles = new_handles
        time.sleep(1e-3)
    return handles  # W0612


if __name__ == '__main__':
    pymanoid.init()
    # viewer.SetBkgndColor([.7, .7, .9])
    pymanoid.get_viewer().SetCamera(
        [[0.43294998, -0.40765653, 0.80397168, -1.7904253],
         [-0.90127634, -0.17995773, 0.39410173, -0.99202776],
         [-0.01597723, -0.89522699, -0.44532388, 1.42718434],
         [0., 0., 0., 1.]])

    plane = pymanoid.Box(
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

    thread = Thread(target=draw_area_thread, args=())
    thread.daemon = True
    thread.start()

    import IPython
    IPython.embed()
