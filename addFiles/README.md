## Add the hammer and nail:

- workspace/src/catkin_data_ws/src/mc_rtc_data/mc_int_obj_description:
    - /meshes: add nail.stl
    - /convex: add nail-ch.txt
    - /urdf: add nail.xml
- workspace/src/catkin_data_ws/src/hrp5_p_description:
    - /convex/HRP5Pmain: add Hammer_mesh-ch.txt
    - /urdf: add the new version of HRP5Pmain.urdf (this vesion adds the hammer to the model of the hrp5p robot)
- workspace/sandbox/hrp5p_mj_description:
    - /xml: upload new version of HRP5Pmain.xml (same as the urdf, adds the hammer at the end of hrp5p's left hand)
    - /meshes: add hammer_scaled.stl to both /visual and /convex
- workspace/src/mc_mujoco/robots: add nail.xml (creates force sensor on the head of the nail)

## Add the table:

```bash
nano ~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml
```
Add the following code to the nano file  

```bash
objects:
  table:
    module: "longtable"
    init_pos:
      translation: [0.8, 0, 0.8]
      rotation: [3.14, 0, 0]
