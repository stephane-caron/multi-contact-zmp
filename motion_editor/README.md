### Motion editor

This script provides a GUI to play motion plans stored as JSON files in the
``plans/`` folder. Each plan provides a sequence of contacts, step durations
and reference ZMP positions at the end of the step, from which controls are
inferred by QP optimization.

To run the motion editor, you will need to launch the associated ROS services:

```bash
roslaunch contact_stability all.launch
```

Once the service is up and running, you can run the setting used in the paper
by:

```bash
python motion_editor.py plans/long-stride.json
```

<img src="https://scaron.info/images/zmp-motion-editor.png" width="500" />

## Note

Motion plans distributed in the ``plans/`` folder were found for HRP-4,
however we do not release its model (copyright issue). We replaced it by
JVRC-1, which has the same kinematic structure but different mass-geometry
(in particular, its center of mass is higher). The plans were not updated
to account for this change in model.
