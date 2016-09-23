# ZMP support areas for multi-contact mobility under frictional constraints

Source code for http://arxiv.org/abs/1510.03232

## Abstract

We propose a method for checking and enforcing multi-contact stability based on
the Zero-tilting Moment Point (ZMP). The key to our development is the
generalization of ZMP *support areas* to take into account (a) frictional
constraints and (b) multiple non-coplanar contacts. We introduce and
investigate two kinds of ZMP support areas. First, we characterize and provide
a fast geometric construction for the support area generated by valid contact
forces, with no other constraint on the robot motion. We call this set the
*full support area*. Next, we consider the control of humanoid robots using the
Linear Pendulum Mode (LPM). We observe that the constraints stemming from the
LPM induce a shrinking of the support area, even for walking on horizontal
floors. We propose an algorithm to compute the new area, which we call
*pendular support area*. We show that, in the LPM, having the ZMP in the
pendular support area is a necessary *and sufficient* condition for contact
stability. Based on these developments, we implement a whole-body controller
and generate feasible multi-contact motions where an HRP-4 humanoid locomotes
in challenging multi-contact scenarios.

<img src="https://scaron.info/images/two-areas.png" height="250" />

Authors:
[Stéphane Caron](https://scaron.info),
[Quang-Cuong Pham](https://www.normalesup.org/~pham/) and
[Yoshihiko Nakamura](http://www.ynl.t.u-tokyo.ac.jp/)

- [Pre-print on the arXiv](http://arxiv.org/abs/1510.03232)
- [Accompanying video](https://scaron.info/videos/tro-2016.mp4)

## Installation

On Ubuntu 14.04, you will first need to [install
OpenRAVE](https://scaron.info/teaching/installing-openrave-on-ubuntu-14.04.html).
Next, install other required dependencies by:

```bash
sudo apt-get install cython python python-dev python-pip python-scipy python-tk
sudo pip install pycddlib quadprog
```

Next, you will need to install the
[contact\_stability](https://github.com/stephane-caron/contact_stability) ROS
package:

```bash
git clone --recursive https://github.com/stephane-caron/contact_stability.git
```

Link it into your Catkin workspace and build. The installation is successful if
you can run ``roslaunch contact_stability all.launch`` without error.

Finally, clone the repository with its submodule:

```bash
git clone --recursive https://github.com/stephane-caron/tro-2016
```

## Usage

There are two scripts in the top directory: ``full_support_area.py`` and
``motion_editor.py``.

### Full support area

Display the full support area for a set of contacts represented by red slabs.
Switch to *object interaction mode* (Escape key) to move contacts around and
observe variations of the area. The area plane can also be changed by zooming
out and moving/tilting the blue box.

### Motion editor

This script provides a GUI to play motion plans stored as JSON files in the
``plans/`` folder. Each plan provides a sequence of contacts, step durations
and reference ZMP positions at the end of the step, from which controls are
inferred by QP optimization.

To run the motion editor, you will need to launch the associated ROS services:

```bash
roslaunch contact_stability all.launch
```

Once the service is up and running, you can do:

```bash
python motion_editor.py
```

See the README in the ``editor/`` folder for instructions on GUI usage.

**NB:** motion plans distributed in the ``plans/`` folder were found for HRP-4,
however we do not release its model (copyright issue). We replaced it by
JVRC-1, which has the same kinematic structure but a different mass-geometry
(in particular, its center of mass is higher). The motion plans were not
updated to account for this change in model.
