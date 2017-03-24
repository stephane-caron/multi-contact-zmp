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

import pymanoid
import simplejson


DEFAULT_CONTACT_COLOR = 'r'
TARGET_COLOR = 'c'


def serialize_array(a):
    return [round(x, 3) for x in a]


class Stance(object):

    def __init__(self, motion_plan, stance_id, stance_dict):
        contact_dict = {}
        for (link, val) in stance_dict['contacts'].iteritems():
            if type(val) is str:
                contact_dict[link] = motion_plan.contacts[val]
            else:  # c is a dict with 'pos' and 'rpy' keys
                contact_dict[link] = motion_plan.create_contact(
                    pos=val['pos'], rpy=val['rpy'],
                    name='Stance-%d-%s' % (stance_id, link),
                    visible=False)
        free_dict = {}
        if 'free' in stance_dict:
            for (link, val) in stance_dict['free'].iteritems():
                if type(val) is str:
                    free_dict[link] = motion_plan.contacts[val]
                else:  # c is a dict with 'pos' and 'rpy' keys
                    free_dict[link] = motion_plan.create_contact(
                        pos=val['pos'], rpy=val['rpy'],
                        name='Stance-%d-Free-%s' % (stance_id, link),
                        color=TARGET_COLOR, visible=False)
                    free_dict[link].set_transparency(0.5)
        if motion_plan.zmp_height is not None:
            # when defined, the zmp_height field overrides stance ZMP's
            stance_dict['zmp'][2] = motion_plan.zmp_height
        self.contacts = pymanoid.ContactSet(contact_dict)
        self.dict_repr = stance_dict
        self.free = pymanoid.ContactSet(free_dict)
        self.stance_id = stance_id
        self.step_duration = stance_dict['step_duration']
        self.zmp = pymanoid.Cube(
            0.02, stance_dict['zmp'], color=TARGET_COLOR,
            name='Stance-%d-ZMP' % stance_id, visible=False)

    def show_targets(self):
        for contact in self.contacts:
            contact.set_color(TARGET_COLOR)
            contact.set_visible(True)
        for free in self.free:
            free.set_color(TARGET_COLOR)
            free.set_visible(True)
        self.zmp.set_visible(True)

    def hide_targets(self):
        for contact in self.contacts:
            contact.set_color(DEFAULT_CONTACT_COLOR)
            contact.set_visible(True)
        for free in self.free:
            is_world_contact = not ('Stance' in free.name)
            if not is_world_contact:
                free.set_visible(False)
        self.zmp.set_visible(False)

    def update_dict_repr(self):
        def update_sub_dict(key, contact_set):
            for (link, val) in self.dict_repr[key].iteritems():
                if type(val) is not str:
                    dc = self.dict_repr[key][link]
                    dc['pos'] = serialize_array(contact_set[link].pos)
                    dc['rpy'] = serialize_array(contact_set[link].rpy)
        update_sub_dict('contacts', self.contacts)
        if 'free' in self.dict_repr:
            update_sub_dict('free', self.free)
        self.dict_repr['comment'] = "Stance %d" % self.stance_id
        self.dict_repr['step_duration'] = self.step_duration
        self.dict_repr['zmp'] = serialize_array(self.zmp.pos)


class MotionPlan(object):

    def __init__(self, path=None, show_targets=False):
        self.path = path
        self.show_targets = show_targets
        self.reset()
        if path is not None:
            self.load(path)

    def reset(self):
        self.X = None
        self.Y = None
        self.com_height = None
        self.contacts = {}
        self.dict_repr = {}
        self.friction = None
        self.progress = -1
        self.stances = []
        self.zmp_height = None
        if 'all_contacts' in self.__dict__:
            for contact in self.all_contacts:
                contact.remove()
        self.all_contacts = []

    def load(self, path):
        with open(path, 'r') as fp:
            d = simplejson.load(fp)
        self.X = d['contact_length'] / 2.
        self.Y = d['contact_width'] / 2.
        self.com_height = d['com_height']
        self.dict_repr = d
        self.friction = d['friction']
        self.path = path
        if 'zmp_height' in d:
            self.zmp_height = d['zmp_height']
        self.contacts = {}
        for (name, c) in d['contacts'].iteritems():
            self.contacts[name] = self.create_contact(
                pos=c['pos'], rpy=c['rpy'],
                name=name, visible=True)
        self.stances = []
        for (i, stance) in enumerate(d['stances']):
            self.stances.append(Stance(self, i, stance))

    def reload(self, path):
        self.reset()
        self.load(path)

    @property
    def cur_stance(self):
        if self.progress < 0:
            return None
        elif self.progress >= self.nb_stances:
            return self.stances[-1]
        return self.stances[self.progress]

    @property
    def prev_stance(self):
        if self.progress - 1 < 0:
            return None
        elif self.progress >= self.nb_stances:
            return self.stances[-2]
        return self.stances[self.progress - 1]

    @property
    def cur_step_duration(self):
        return self.cur_stance.step_duration

    @property
    def nb_stances(self):
        """Number of stances in the plan."""
        return len(self.stances)

    @property
    def started(self):
        """Have we started to follow the motion plan?"""
        return self.progress >= 0

    @property
    def init_stance(self):
        if self.nb_stances <= 0:
            return None
        return self.stances[0]

    def start(self):
        if self.started:
            return
        self.step()

    def stop(self):
        if not self.started:
            return
        self.cur_stance.hide_targets()
        self.progress = -1

    def step(self):
        """
        Go to the next stance. Returns False when there is no such stance.
        """
        if self.progress >= self.nb_stances:
            return False
        if self.progress >= 0:
            self.cur_stance.hide_targets()
        self.progress += 1
        if self.show_targets:
            self.cur_stance.show_targets()
        return True

    def step_back(self):
        """
        Go to the previous stance. Returns False when there is no such stance.
        """
        if self.progress <= 0:
            return False
        self.cur_stance.hide_targets()
        self.progress -= 1
        if self.show_targets:
            self.cur_stance.show_targets()
        return True

    def create_contact(self, pos, rpy, name, color=DEFAULT_CONTACT_COLOR,
                       visible=True):
        contact = pymanoid.Contact(
            X=self.X, Y=self.Y, pos=pos, rpy=rpy,
            friction=self.friction, color=color, name=name, visible=visible)
        self.all_contacts.append(contact)
        return contact

    def dump(self, path):
        self.update_dict_repr()
        with open(path, 'w') as fp:
            simplejson.dump(self.dict_repr, fp, indent=4, sort_keys=True)
        print "Saved motion plan to", path

    def update_dict_repr(self):
        for (name, contact) in self.contacts.iteritems():
            dc = self.dict_repr['contacts'][name]
            dc['pos'] = serialize_array(contact.pos)
            dc['rpy'] = serialize_array(contact.rpy)
        for (i, stance) in enumerate(self.stances):
            stance.update_dict_repr()
        self.dict_repr['contact_length'] = 2 * self.X
        self.dict_repr['contact_width'] = 2 * self.Y
        self.dict_repr['friction'] = self.friction

    def toggle_targets(self):
        self.show_targets = not self.show_targets
        if self.show_targets:
            self.cur_stance.show_targets()
        else:
            self.cur_stance.hide_targets()
