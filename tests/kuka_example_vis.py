import os, sys
import numpy as np
import pytorch_kinematics as pk

from pytorch_kinematics.visualizer import Visualizer

model_path = os.path.join(os.environ["HOME"], "stash/pytorch_kinematics/tests")

chain = pk.build_serial_chain_from_urdf(
    open(os.path.join(model_path, "kuka_iiwa.urdf")).read(), "lbr_iiwa_link_7"
)
print(chain)
print(chain.get_joint_parameter_names())
th = [0.0, -np.pi / 4.0, 0.0, np.pi / 2.0, 0.0, np.pi / 4.0, 0.0]
ret = chain.forward_kinematics(th, end_only=False)
print(ret)
viz = Visualizer()
viz.add_robot(ret, chain.visuals_map(), mesh_file_path=model_path, axes=True)
viz.spin()
