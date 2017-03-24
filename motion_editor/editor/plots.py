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

import pylab

from pymanoid.toolbox import plot_polygon
from pymanoid.draw import _convert_cone2d_to_vertices


class PlotData(object):

    def __init__(self):
        pylab.ion()
        self.com_real = []
        self.com_ref = []
        self.support_areas = []
        self.xlabel = "$y$ (m)"
        self.ylabel = "$x$ (m)"
        self.xlim = (-0.6, 0.1)
        self.ylim = (0. - 0.05, 1.4 + 0.05)
        self.zmp_real = []
        self.zmp_ref = []

    def append(self, zmp_ref, com_ref, zmp_real, com_real):
        self.com_real.append(com_real)
        self.com_ref.append(com_ref)
        self.zmp_real.append(zmp_real)
        self.zmp_ref.append(zmp_ref)

    def append_support_area(self, area):
        self.support_areas.append(area)

    def plot_com(self):
        pylab.plot(
            [-p[1] for p in self.com_real], [p[0] for p in self.com_real],
            'g-', lw=2)
        pylab.plot(
            [-p[1] for p in self.com_ref], [p[0] for p in self.com_ref],
            'k--', lw=1)
        pylab.legend(('$p_G$', '$p_G^{ref}$'), loc='upper right')
        pylab.grid(False)
        pylab.xlim(self.xlim)
        pylab.ylim(self.ylim)
        pylab.xlabel(self.xlabel)
        pylab.ylabel(self.ylabel)
        pylab.title("COM trajectory")

    def plot_zmp(self):
        pylab.plot(
            [-p[1] for p in self.zmp_real], [p[0] for p in self.zmp_real],
            'r-', lw=2)
        pylab.plot(
            [-p[1] for p in self.zmp_ref], [p[0] for p in self.zmp_ref],
            'k--', lw=1)
        pylab.legend(('$p_Z$', '$p_Z^{ref}$'), loc='upper right')
        pylab.grid(False)
        pylab.xlim(self.xlim)
        pylab.ylim(self.ylim)
        pylab.xlabel(self.xlabel)
        pylab.ylabel(self.ylabel)
        pylab.title("ZMP trajectory")

    def plot_trajectories(self):
        pylab.clf()
        pylab.rc('text', usetex=True)
        pylab.rc('font', size=18)
        pylab.subplot(121)
        self.plot_com()
        pylab.subplot(122)
        self.plot_zmp()

    def plot_support_areas(self, indices=None):
        if indices is None:
            indices = range(len(self.support_areas))
        for i in indices:
            vertices, rays = self.support_areas[i]
            vertices_3d = _convert_cone2d_to_vertices(vertices, rays)
            vertices = [[-v[1], v[0]] for v in vertices_3d]
            plot_polygon(vertices, color='g', fill=None, lw=2)
        pylab.xlim(self.xlim)
        pylab.ylim(self.ylim)
