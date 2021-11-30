from typing import Dict, List, Tuple, Optional, Union
import os, sys

import numpy as np

# import transformations as tf
import pytorch_kinematics.pytorch3d.transforms as tf
import vtk
from vtk.util.colors import tomato

from pytorch_kinematics import frame  # , transforms
import pytorch_kinematics as pk

# for euler_from_quaternion:
import transformations as tf2

# use kinpy Transform to construct Visualizer:
# sys.path.append(os.path.join(os.environ["HOME"], "stash/kinpy/kinpy"))
from .kinpy_utils import transform


def quat_pos_from_transform3d(tg):
    m = tg.get_matrix()
    pos = m[:, :3, 3]
    rot = pk.matrix_to_quaternion(m[:, :3, :3])
    return pos, rot


class Visualizer(object):
    def __init__(self, win_size: Tuple[int, int] = (640, 480)) -> None:
        self._ren = vtk.vtkRenderer()
        self._ren.SetBackground(0.1, 0.2, 0.4)
        self._win = vtk.vtkRenderWindow()
        self._win.SetSize(*win_size)
        self._win.AddRenderer(self._ren)
        self._inter = vtk.vtkRenderWindowInteractor()
        self._inter.SetRenderWindow(self._win)

    def add_robot(
        self,
        transformations: Union[Dict[str, np.ndarray], List[Dict[str, np.ndarray]]],
        visuals_map: Dict[str, frame.Visual],
        mesh_file_path: str = "./",
        axes: bool = False,
    ) -> None:
        if isinstance(transformations, dict):
            transformations = [transformations]
        self.transform_idx = 0
        self.transformations = transformations
        self.visuals_map = visuals_map
        self.axes = axes

        for k, trans in transformations[0].items():
            if axes:
                pos, rot = quat_pos_from_transform3d(trans)
                pos = pos.detach().numpy()
                rot = rot.detach().numpy()
                tf = transform.Transform(rot[0], pos[0])

                self.add_axes(tf)
            for v in visuals_map[k]:
                tfo = trans.compose(v.offset)
                pos, rot = quat_pos_from_transform3d(tfo)
                pos = pos.detach().numpy()
                rot = rot.detach().numpy()
                tf = transform.Transform(rot[0], pos[0])

                # tf = trans * v.offset
                if v.geom_type == "mesh":
                    self.add_mesh(os.path.join(mesh_file_path, v.geom_param), tf)
                elif v.geom_type == "cylinder":
                    self.add_cylinder(v.geom_param[0], v.geom_param[1], tf)
                elif v.geom_type == "box":
                    self.add_box(v.geom_param, tf)
                elif v.geom_type == "sphere":
                    self.add_sphere(v.geom_param, tf)
                elif v.geom_type == "capsule":
                    self.add_capsule(v.geom_param[0], v.geom_param[1], tf)

    def add_shape_source(
        self, source: vtk.vtkAbstractPolyDataReader, transform: transform.Transform
    ) -> None:
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(tomato)
        actor.SetPosition(transform.pos)
        rpy = np.rad2deg(tf2.euler_from_quaternion(transform.rot, "rxyz"))
        actor.RotateX(rpy[0])
        actor.RotateY(rpy[1])
        actor.RotateZ(rpy[2])
        self._ren.AddActor(actor)

    def add_axes(self, trans: transform.Transform) -> None:
        transform = vtk.vtkTransform()
        transform.Translate(trans.pos)
        rpy = np.rad2deg(tf2.euler_from_quaternion(trans.rot, "rxyz"))
        transform.RotateX(rpy[0])
        transform.RotateY(rpy[1])
        transform.RotateZ(rpy[2])
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1, 0.1, 0.1)
        axes.AxisLabelsOff()
        axes.SetUserTransform(transform)
        self._ren.AddActor(axes)

    def load_obj(self, filename: str) -> None:
        reader = vtk.vtkOBJReader()
        reader.SetFileName(filename)
        return reader

    def load_ply(self, filename: str) -> None:
        reader = vtk.vtkPLYReader()
        reader.SetFileName(filename)
        return reader

    def load_stl(self, filename: str) -> None:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
        return reader

    def add_cylinder(
        self,
        radius: float,
        height: float,
        tf: transform.Transform = transform.Transform(),
    ):
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetResolution(20)
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        self.add_shape_source(cylinder, tf)

    def add_box(
        self, size: List[float], tf: transform.Transform = transform.Transform()
    ) -> None:
        cube = vtk.vtkCubeSource()
        cube.SetXLength(size[0])
        cube.SetYLength(size[1])
        cube.SetZLength(size[2])
        self.add_shape_source(cube, tf)

    def add_sphere(
        self, radius: float, tf: transform.Transform = transform.Transform()
    ) -> None:
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(radius)
        self.add_shape_source(sphere, tf)

    def add_capsule(
        self,
        radius: float,
        fromto: np.ndarray,
        tf: transform.Transform = transform.Transform(),
        step: float = 0.05,
    ) -> None:
        for t in np.arange(0.0, 1.0, step):
            trans = transform.Transform(pos=t * fromto[:3] + (1.0 - t) * fromto[3:])
            self.add_sphere(radius, tf * trans)

    def add_mesh(
        self, filename: str, tf: transform.Transform = transform.Transform()
    ) -> None:
        _, ext = os.path.splitext(filename)
        ext = ext.lower()
        if ext == ".stl":
            reader = self.load_stl(filename)
        elif ext == ".obj":
            reader = self.load_obj(filename)
        elif ext == ".ply":
            reader = self.load_ply(filename)
        else:
            raise ValueError("Unsupported file extension, '%s'." % ext)
        self.add_shape_source(reader, tf)

    def update_robot_pose(
        self,
        index_step: int = 1,
    ) -> None:
        """Step thru list of forward kinetic transformation poses as provided to the
        add_robot method.  It uses the same method of updating the pose as used during
        setup: SetPosition and RotateX|Y|Z.  The axes are not updated.

        Args:
          index_step (int): distance to move along transformation list to next pose
        """

        self.transform_idx += index_step
        self.transform_idx %= len(self.transformations)  # cycle thru list
        print(f"Transform index: {self.transform_idx}")

        next_transform = self.transformations[self.transform_idx]

        link_names = list(self.visuals_map.keys())

        actors = self._ren.GetActors()

        actor_iterator = actors.NewIterator()
        for indx, actor in enumerate(actor_iterator):
            # If present, skip 6 axes actors per link:
            if not self.axes or indx % 7 == 6:
                l_indx = indx if not self.axes else int(indx / 7)
                l_indx += 1

                link_name = link_names[l_indx]

                v = self.visuals_map[link_name][0]  # this assumes 1 visual per link
                trans = next_transform[link_name]
                tfo = trans.compose(v.offset)
                pos, rot = quat_pos_from_transform3d(tfo)
                pos = pos.detach().numpy()
                rot = rot.detach().numpy()
                tf = transform.Transform(rot[0], pos[0])

                actor.SetPosition(tf.pos)

                rpy = np.rad2deg(tf2.euler_from_quaternion(tf.rot, "rxyz"))
                # Reset orientation, then rotate to new orientation:
                actor.SetOrientation(0.0, 0.0, 0.0)
                actor.RotateX(rpy[0])
                actor.RotateY(rpy[1])
                actor.RotateZ(rpy[2])

        self._win.Render()

    def key_press(self, obj, event):
        key = obj.GetKeySym()

        if key == "Up" or key == "Right":
            self.update_robot_pose(-1)
        elif key == "Down" or key == "Left":
            self.update_robot_pose(1)

    def spin(self) -> None:
        self._win.Render()
        self._inter.Initialize()
        self._inter.Start()
