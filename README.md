# Multi-contact ZMP support areas

Source code for http://arxiv.org/pdf/1510.03232.pdf

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
package. Link it into your Catkin workspace and build. The installation is successful if
you can run ``roslaunch contact_stability all.launch`` without error. Finally, clone the
repository with its submodules:

```bash
git clone --recursive https://github.com/stephane-caron/tro-2016
```

## Usage

There are three sub-folders corresponding to different developments of the paper:

- [full\_support\_area](full_support_area/) for the Full Support Area (Section
  III of the paper)
- [motion\_editor](motion_editor/) for the motion generation framework (Section
  V) based on the Pendular Support Area (Section IV)
- [n\_moment\_point](n_moment_point/) for the n-Moment Point (Appendix A)

<a href="motion_editor/"><img src="https://scaron.info/images/zmp-motion-editor.png" width="100%" /></a>
*Screenshot of the motion editor*
