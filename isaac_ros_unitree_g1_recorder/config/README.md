# `config/` — Recorder + Embodiment Configuration

## `recorder_params.yaml`

Default ROS parameters for the recorder node (sync rate, output dir, topic
names, camera shape).

## `g1_joints.json`

The canonical G1 joint-name layout for both the **observation state** and
the **policy action** that the MCAP-to-LeRobot converter writes.

The file is consumed at LEAPP-export time via the `--joint_config <path>`
flag of the `export_with_leapp.py` script from `nvidia-isaac/gr00t-leapp-export`
(branch `n17-export`). Without
it, the export tool falls back to placeholder element names
(`left_leg_0`, `left_leg_1`, ...) that the deploy stack's `InputBuilder`
cannot resolve against `/joint_states`. See
`docs/.../tutorial_leapp_export.rst`.

### Schema

```jsonc
{
  "state":  { "<group>": ["<joint_name>", ...], ... },
  "action": { "<group>": ["<joint_name>", ...], ... }
}
```

* Each `<group>` corresponds to a body-part slice the GR00T modality
  config refers to: `left_leg`, `right_leg`, `waist`, `left_arm`,
  `right_arm`, `left_hand`, `right_hand`.
* The `action` block additionally includes `navigate_command` and
  `base_height_command` for teleop-locomotion targets that are not
  present in `/joint_states`.
* All joint names match the `name[]` field of
  `sensor_msgs/JointState` published by `joint_state_broadcaster` on
  the G1.

### Order matters

The order of names within each list is **not just a label**. The
deploy `InputBuilder` uses each name to look up its value in the live
`/joint_states` message, then assigns those values to the model's
input tensor at the position dictated by their order in the list.
The model was trained on parquet columns laid out in this exact
order; if the order in this JSON disagrees with the training-time
column order, the deploy stack feeds the model the right joint
*values* but at the wrong tensor *positions* — silently producing
wrong action predictions with no error. Always treat order as
load-bearing.

### Asymmetric hand orderings

`state.right_hand` is intentionally **asymmetric** versus
`state.left_hand` (left uses thumb-middle-index, right uses
thumb-index-middle). This mirrors the canonical layout the
GR00T-flavored unitree_g1 embodiment was trained against and matches
`G1_CANONICAL_STATE_JOINT_ORDER` in the MCAP-to-LeRobot converter.

`action.left_hand` and `action.right_hand` use the **index-middle-thumb**
ordering (symmetric across hands), matching
`G1_CANONICAL_ACTION_JOINT_ORDER` in the converter.

### Locomotion command labels

`action.navigate_command` (3-D: `nav_vx`, `nav_vy`, `nav_vyaw`) and
`action.base_height_command` (1-D: `base_height`) are not joints but
control-target labels for the policy's locomotion outputs. The
recorder captures them from `/xr_teleop/root_twist` and
`/xr_teleop/root_pose`; the converter writes them as
`action.navigate_command` and `action.base_height_command` columns.
The labels are present in this JSON for symmetry with the joint
groups so that `--joint_config` registers all action keys the
modality config might reference.
