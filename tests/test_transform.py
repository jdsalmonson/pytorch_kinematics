import torch
import pytorch_kinematics.pytorch3d.transforms as tf


def test_transform():
    N = 20
    mats = tf.random_rotations(N, dtype=torch.float64, device="cpu", requires_grad=True)
    assert list(mats.shape) == [N, 3, 3]
    # test batch conversions
    quat = tf.matrix_to_quaternion(mats)
    assert list(quat.shape) == [N, 4]
    mats_recovered = tf.quaternion_to_matrix(quat)
    assert torch.allclose(mats, mats_recovered)

    quat_identity = tf.quaternion_multiply(quat, tf.quaternion_invert(quat))
    assert torch.allclose(
        tf.quaternion_to_matrix(quat_identity),
        torch.eye(3, dtype=torch.float64).repeat(N, 1, 1),
    )


def test_translations():
    t = tf.Translate(1, 2, 3)
    points = torch.tensor([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.5, 0.5, 0.0]]).view(
        1, 3, 3
    )
    points_out = t.transform_points(points)
    points_out_expected = torch.tensor(
        [[2.0, 2.0, 3.0], [1.0, 3.0, 3.0], [1.5, 2.5, 3.0]]
    ).view(1, 3, 3)
    assert torch.allclose(points_out, points_out_expected)

    N = 20
    points = torch.randn((N, N, 3))
    translation = torch.randn((N, 3))
    transforms = tf.Transform3d(pos=translation)
    translated_points = transforms.transform_points(points)
    assert torch.allclose(
        translated_points, translation.repeat(N, 1, 1).transpose(0, 1) + points
    )
    returned_points = transforms.inverse().transform_points(translated_points)
    assert torch.allclose(returned_points, points, atol=1e-6)


def test_rotate_axis_angle():
    t = tf.Transform3d().rotate_axis_angle(90.0, axis="Z")
    points = torch.tensor([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 1.0]]).view(
        1, 3, 3
    )
    normals = torch.tensor([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]).view(
        1, 3, 3
    )
    points_out = t.transform_points(points)
    normals_out = t.transform_normals(normals)
    points_out_expected = torch.tensor(
        [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [-1.0, 0.0, 1.0]]
    ).view(1, 3, 3)
    normals_out_expected = torch.tensor(
        [[0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
    ).view(1, 3, 3)
    assert torch.allclose(points_out, points_out_expected)
    assert torch.allclose(normals_out, normals_out_expected)


def test_rotate():
    R = tf.so3_exp_map(torch.randn((1, 3)))
    t = tf.Transform3d().rotate(R)
    points = torch.tensor([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.5, 0.5, 0.0]]).view(
        1, 3, 3
    )
    normals = torch.tensor([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]).view(
        1, 3, 3
    )
    points_out = t.transform_points(points)
    normals_out = t.transform_normals(normals)
    points_out_expected = torch.bmm(points, R)
    normals_out_expected = torch.bmm(normals, R)
    assert torch.allclose(points_out, points_out_expected)
    assert torch.allclose(normals_out, normals_out_expected)


if __name__ == "__main__":
    test_transform()
    test_translations()
    test_rotate_axis_angle()
    test_rotate()
