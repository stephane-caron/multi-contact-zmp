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

from numpy import array, eye, dot, zeros, hstack, vstack, ones, cos, sin, sqrt
from pymanoid.toolbox import cvxopt_solve_qp
from scipy.linalg import block_diag


class Controller(object):

    pass


class PendulumController(Controller):

    def __init__(self, x_init, xd_init, x_goal, xd_goal, omega2, duration,
                 timesteps):
        """
        Initialize the controller for a linear pendulum with control law:

            xdd = omega^2 (p - x)

        where p is the fulcrum position.

        INPUT:

        - ``x_init`` -- initial position along axis
        - ``xd_init`` -- initial velocity along axis
        - ``x_goal`` -- target position along axis
        - ``xd_goal`` -- target velocity along axis
        - ``omega2`` -- positive feedback gain
        - ``duration`` -- duration of the motion
        - ``timesteps`` -- number of states in the discrete approximation
        """
        K, dt = timesteps, duration / timesteps
        omega = sqrt(omega2)
        A = array([
            [cos(omega * dt), sin(omega * dt) * 1. / omega],
            [-omega * sin(omega * dt), cos(omega * dt)]])
        v = array([
            1 - cos(omega * dt),
            omega * sin(omega * dt)]).reshape((2, 1))
        # Linearized version:
        # A = array([
        #     [1., dt],
        #     [-omega * omega * dt, 1.]])
        # v = array([
        #     0.,
        #     omega * omega * dt]).reshape((2, 1))
        A_pow = eye(2)
        Phi_list = [eye(2)]
        Psi_list = [zeros((2, K))]
        while len(Psi_list) <= K:
            d0 = v.reshape((2, 1))
            d1 = Psi_list[-1][:, :-1]
            Psi_list.append(hstack([d0, d1]))
            v = dot(A, v)
            A_pow = dot(A, A_pow)
            Phi_list.append(A_pow)
        Phi_last = Phi_list.pop()
        Psi_last = Psi_list.pop()
        Phi = vstack(Phi_list)
        Psi = vstack(Psi_list)

        assert Phi.shape == (2 * K, 2)
        assert Psi.shape == (2 * K, K)
        assert omega2 > 0

        self.I = eye(K)
        self.K = K
        self.Phi = Phi
        self.Psi = Psi
        self.Phi_last = Phi_last
        self.Psi_last = Psi_last
        self.X_target = array([x_goal, xd_goal])
        self.X_init = array([x_init, xd_init])
        self.p_max = max(x_init, x_goal) * ones(K)
        self.p_min = min(x_init, x_goal) * ones(K)
        self.dt = dt
        self.computed_control = self.compute_control()

    def compute_control(self):
        """
        Compute the optimal control p=[p[0] ... p[K]].
        """
        K, Phi, Psi, X_init = self.K, self.Phi, self.Psi, self.X_init

        # Cost 1: sum_i 1/K * (x_i - p_i) ** 2
        B0 = array([[1, 0], [0, 0]])
        C0 = array([1, 0])
        B = block_diag(*[B0 for i in xrange(K)])
        C = block_diag(*[C0 for i in xrange(K)])
        P1 = 1. / K * (dot(Psi.T, dot(B, Psi)) - 2 * dot(C, Psi) + eye(K))
        q1 = 1. / K * dot(dot(Phi, X_init).T, dot(B, Psi) - C.T)

        # Cost 2: sum_i (p_i - p_{i - 1}) ** 2
        N1, N2 = zeros((K, K)), zeros((K, K))
        N1[0:K - 1, 0:K - 1] = eye(K - 1)
        N2[0:K - 1, 1:K] = eye(K - 1)
        P2 = dot((N2 - N1).T, (N2 - N1))
        q2 = zeros(K)

        w1, w2 = 1., 100.
        P = w1 * P1 + w2 * P2
        q = w1 * q1 + w2 * q2
        G = vstack([self.I, -self.I])
        h = hstack([self.p_max, -self.p_min])
        A = vstack([self.Psi_last, hstack([zeros(K - 1), [1]])])
        b = hstack([self.X_target, [self.X_target[0]]]) \
            - hstack([dot(self.Phi_last, X_init), [0]])

        p = cvxopt_solve_qp(P, q, G, h, A, b)
        return p

    def control(self, t):
        assert self.computed_control is not None
        i = int(t / self.dt)
        p = self.computed_control
        return p[i] if i < p.shape[0] else self.X_target[0]


class StationaryController(Controller):

    def __init__(self, value):
        self.value = value

    def control(self, t):
        return self.value
