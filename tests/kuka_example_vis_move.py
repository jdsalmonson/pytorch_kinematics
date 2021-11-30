import os, sys
import rich
import numpy as np
import torch as th
import pytorch_kinematics as pk
import pytorch_kinematics.pytorch3d.transforms as tf

from pytorch_kinematics.visualizer import Visualizer

model_path = os.getcwd()

chain = pk.build_serial_chain_from_urdf(
    open(os.path.join(model_path, "kuka_iiwa.urdf")).read(), "lbr_iiwa_link_7"
)
print(chain)

angs = th.tensor(
    [0.0, 0.0 * -np.pi / 4.0, 0.0, np.pi / 2.0, 0.0, np.pi / 4.0, 0.0],
    # (len(chain.get_joint_parameter_names()) - 2) * [0.0] + [np.pi / 4, np.pi / 3.0],
    # [0.001] + (len(chain.get_joint_parameter_names()) - 1) * [0.0],
    # len(chain.get_joint_parameter_names()) * [0.0],
    # [0.001, np.pi / 4] + (len(chain.get_joint_parameter_names()) - 2) * [0.001],
    # requires_grad=True,
)

# First pose:
fk = chain.forward_kinematics(angs, end_only=False)

# Add poses to sequence:
fkl = [fk]
new_angs = angs
for _ in range(10):
    new_angs = new_angs.clone()
    new_angs[1] += 0.1
    new_angs[2] -= 0.2
    new_angs[4] -= 0.2
    new_angs[5] += 0.1
    fkl.append(chain.forward_kinematics(new_angs, end_only=False))

viz = Visualizer()
viz.add_robot(
    fkl,
    chain.visuals_map(),
    mesh_file_path=model_path,
    axes=False,
)

# Add key_press callback, then render:
print(
    'Press "up" or "forward" arrows to advance poses,\n'
    'and "down" or "back" arrows to go back'
)
viz._inter.AddObserver("KeyPressEvent", viz.key_press)
viz.spin()
