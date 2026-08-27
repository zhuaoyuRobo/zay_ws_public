# Simulation topic contract

The public integration package expects an external simulator or sensor source.
No private scene, map, policy, or experiment asset is bundled.

## Required navigation inputs

- `/odom` or the odometry topic selected by the launch arguments
- `/scan` when `synthetic_scan:=false`
- a user-supplied Nav2 map YAML passed as `map:=...`

## Optional learned locomotion inputs

- `/joint_states`
- `/imu`
- a policy checkpoint passed as `policy_path:=...`
- the matching environment description passed as `env_config_path:=...`

The policy action scale defaults to zero. The caller must verify observation
ordering, joint ordering, action scaling, coordinate frames, and command limits
against the chosen simulator before enabling policy output.

## Outputs

- Nav2 velocity commands on the configured command topic
- optional joint commands on `/joint_command`
- optional bounded robot commands through `go2w_cmd_vel_bridge`

The hardware bridge starts with `enable_output:=false`. The tracked speed
limits are conservative public examples and are not calibrated robot values.
