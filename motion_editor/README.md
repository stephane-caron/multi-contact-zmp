# ZMP motion editor

This script provides a GUI to play motion plans stored as JSON files in the
``plans/`` folder. Each plan provides a sequence of contacts, step durations
and reference ZMP positions at the end of the step, from which controls are
inferred by QP optimization.

To run the motion editor, you will need to launch the associated ROS services:

```bash
roslaunch contact_stability all.launch
```

Once the service is up and running, you can run the scenario used in the paper (Section V.B)
by:

```bash
python motion_editor.py plans/long-stride.json
```

Two windows should pop up: the OpenRAVE GUI, and a side panel:

<img src="https://scaron.info/images/zmp-motion-editor.png" width="100%" />

## Side panel

The side panel includes the following buttons:

### Line 1: come and go

- *Open*: open a new model plan from a JSON file (see [plans/](plans/) for examples)
- *Save as*: save current plan in a different JSON file
- *Restart*: reload plan and reset robot configuration
- *Robot transp.*: change robot transparency between opaque, transparent and hidden
- *Quit*: clear

### Line 2: plan edition

- *<*: rewind to the previous stance in the plan
- *>*: fast-forward to the next stance in the plan
- *Plan next step* (in *Step* mode): finish current step and go to *Plan* mode; this mode allows you to edit the targets (contact locations, target ZMP) of the next step
- *Step* (in *Plan* mode): exit *Plan* mode and execute the step
- *Play*: when checked, skips *Plan* mode at the end of a step and execute the plan without interruption

### Line 3: displays

- *Show plan*: show the initial motion plan in ZMP plane
- *Show COM polygon*: show the static-equilibrium COM area
- *Show ZMP areas*: show both the full and pendular ZMP support areas
- *Show targets* (in *Plan* mode): shows targets (contact locations, target ZMP) for the next step in cyan
- *Show forces*: validate contact-stability of the motion by computing supporting contact forces

## NB: motions will look weird due to wrong robot model

Motion plans distributed in the [plans/](plans/) folder were found for HRP-4,
however we do not release its model (copyright issue). We replaced it by
JVRC-1, which has the same kinematic structure but different mass-geometry
(in particular, its center of mass is higher). The plans were not updated
to account for this change in model.
