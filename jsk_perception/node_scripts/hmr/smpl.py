from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import itertools, pkg_resources, sys
from distutils.version import LooseVersion
if LooseVersion(pkg_resources.get_distribution("chainer").version) >= LooseVersion('7.0.0') and \
   sys.version_info.major == 2:
   print('''Please install chainer <= 7.0.0:

    sudo pip install chainer==6.7.0

c.f https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485
''', file=sys.stderr)
   sys.exit(1)
if [p for p in list(itertools.chain(*[pkg_resources.find_distributions(_) for _ in sys.path])) if "cupy-" in p.project_name ] == []:
   print('''Please install CuPy

    sudo pip install cupy-cuda[your cuda version]
i.e.
    sudo pip install cupy-cuda91

''', file=sys.stderr)
   sys.exit(1)
from chainer import Parameter
from chainer import Variable
import chainer
import chainer.functions as F
import numpy as np


def batch_global_rigid_transformation(Rs, Js, parent, rotate_base=False):
    """
    Computes absolute joint locations given pose.

    rotate_base: if True, rotates the global rotation by 90 deg in x axis.
    if False, this is the original SMPL coordinate.

    Args:
      Rs: N x 24 x 3 x 3 rotation vector of K joints
      Js: N x 24 x 3, joint locations before posing
      parent: 24 holding the parent id for each index

    Returns
      new_J : `Tensor`: N x 24 x 3 location of absolute joints
      A     : `Tensor`: N x 24 4 x 4 relative joint transformations for LBS.
    """
    xp = Rs.xp
    N = Rs.shape[0]
    if rotate_base:
        print('Flipping the SMPL coordinate frame!!!!')
        rot_x = Variable(
            [[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=Rs.dtype)
        rot_x = F.reshape(F.tile(rot_x, [N, 1]), [N, 3, 3])
        root_rotation = F.matmul(Rs[:, 0, :, :], rot_x)
    else:
        root_rotation = Rs[:, 0, :, :]

    # Now Js is N x 24 x 3 x 1
    Js = F.expand_dims(Js, -1)

    def make_A(R, t, name=None):
        # Rs is N x 3 x 3, ts is N x 3 x 1
        R_homo = F.pad(R, [[0, 0], [0, 1], [0, 0]], 'constant')
        t_homo = F.concat([t, xp.ones([N, 1, 1], 'f')], 1)
        return F.concat([R_homo, t_homo], 2)

    A0 = make_A(root_rotation, Js[:, 0])
    results = [A0]
    for i in range(1, parent.shape[0]):
        j_here = Js[:, i] - Js[:, parent[i]]
        A_here = make_A(Rs[:, i], j_here)
        res_here = F.matmul(
            results[parent[i]], A_here)
        results.append(res_here)

    # 10 x 24 x 4 x 4
    results = F.stack(results, axis=1)

    new_J = results[:, :, :3, 3]

    # --- Compute relative A: Skinning is based on
    # how much the bone moved (not the final location of the bone)
    # but (final_bone - init_bone)
    # ---
    Js_w0 = F.concat([Js, xp.zeros([N, 24, 1, 1], 'f')], 2)
    init_bone = F.matmul(results, Js_w0)
    # Append empty 4 x 3:
    init_bone = F.pad(init_bone, [[0, 0], [0, 0], [0, 0], [3, 0]], 'constant')
    A = results - init_bone

    return new_J, results


def batch_skew(vec, batch_size=None):
    """
    vec is N x 3, batch_size is int

    returns N x 3 x 3. Skew_sym version of each matrix.
    """
    xp = vec.xp
    if batch_size is None:
        batch_size = vec.shape[0]
    col_inds = xp.array([1, 2, 3, 5, 6, 7])
    indices = F.reshape(
        F.repeat(col_inds.reshape(1, -1), batch_size, axis=0) +
        F.repeat(F.reshape(xp.arange(0, batch_size) * 9, [-1, 1]), 6, axis=1),
        [-1, 1])
    updates = F.reshape(
        F.stack(
            [-vec[:, 2], vec[:, 1], vec[:, 2], -vec[:, 0], -vec[:, 1],
             vec[:, 0]], axis=1), [-1])
    res = Variable(xp.zeros((batch_size * 3 * 3), 'f'))
    res.data[indices.reshape(-1).data] = updates.data
    res = F.reshape(res, [batch_size, 3, 3])
    return res


def batch_rodrigues(theta):
    """
    Theta is N x 3
    """
    batch_size = theta.shape[0]
    xp = theta.xp

    angle = F.expand_dims(F.sqrt(F.batch_l2_norm_squared(theta + 1e-8)), -1)
    r = F.expand_dims(theta / F.tile(angle, 3), -1)

    angle = F.expand_dims(angle, -1)
    cos = F.cos(angle)
    sin = F.sin(angle)
    cos = F.tile(cos, (3, 3))
    sin = F.tile(sin, (3, 3))

    outer = F.matmul(r, r, transb=True)

    eyes = F.tile(F.expand_dims(
        Variable(xp.array(xp.eye(3), 'f')), 0), (batch_size, 1, 1))
    R = cos * eyes + (1 - cos) * outer + sin * batch_skew(r, batch_size)
    return R


class SMPL(chainer.Chain):

    def __init__(self):
        super(SMPL, self).__init__()
        self.parents = np.array([
            4294967295, 0, 0, 0, 1,
            2, 3, 4, 5, 6, 7, 8, 9, 9, 9,
            12, 13, 14, 16, 17, 18, 19, 20, 21], 'i')

        with self.init_scope():
            self.v_template = Parameter(0, (6890, 3))
            self.size = [self.v_template.shape[0], 3]
            self.shapedirs = Parameter(0, (10, 20670))
            self.J_regressor = Parameter(0, (6890, 24))
            self.posedirs = Parameter(0, (207, 20670))
            self.weights = Parameter(0, (6890, 24))
            self.joint_regressor = Parameter(0, (6890, 19))

    def __call__(self, beta, theta, get_skin=False, with_a=False):
        batch_size = beta.shape[0]

        # 1. Add shape blend shapes
        # (N x 10) x (10 x 6890*3) = N x 6890 x 3
        self.beta_shapedirs = F.matmul(beta, self.shapedirs)
        v_shaped = F.reshape(
            F.matmul(beta, self.shapedirs),
            [-1, self.size[0], self.size[1]]) + \
            F.repeat(self.v_template[None, ], batch_size, axis=0)
        self.v_shaped = v_shaped

        # 2. Infer shape-dependent joint locations.
        Jx = F.matmul(v_shaped[:, :, 0], self.J_regressor)
        Jy = F.matmul(v_shaped[:, :, 1], self.J_regressor)
        Jz = F.matmul(v_shaped[:, :, 2], self.J_regressor)
        J = F.stack([Jx, Jy, Jz], axis=2)

        self.J = J

        # 3. Add pose blend shapes
        # N x 24 x 3 x 3
        Rs = F.reshape(
            batch_rodrigues(F.reshape(theta, [-1, 3])), [-1, 24, 3, 3])
        self.Rs = Rs
        # Ignore global rotation.
        pose_feature = F.reshape(Rs[:, 1:, :, :] -
                                 F.repeat(F.repeat(Variable(self.xp.array(self.xp.eye(3), 'f'))[
                                          None, ], 23, axis=0)[None, ], batch_size, axis=0),
                                 [-1, 207])
        self.pose_feature = pose_feature

        # (N x 207) x (207, 20670) -> N x 6890 x 3
        v_posed = F.reshape(
            F.matmul(pose_feature, self.posedirs),
            [-1, self.size[0], self.size[1]]) + v_shaped

        # 4. Get the global joint location
        self.J_transformed, A = batch_global_rigid_transformation(
            Rs, J, self.parents)

        # 5. Do skinning:
        # W is N x 6890 x 24
        W = F.reshape(
            F.tile(self.weights, (batch_size, 1)), [batch_size, -1, 24])
        # (N x 6890 x 24) x (N x 24 x 16)
        T = F.reshape(
            F.matmul(W, F.reshape(A, [batch_size, 24, 16])),
            [batch_size, -1, 4, 4])
        v_posed_homo = F.concat(
            [v_posed, self.xp.ones([batch_size, v_posed.shape[1], 1], 'f')], 2)
        v_homo = F.matmul(T, F.expand_dims(v_posed_homo, -1))

        verts = v_homo[:, :, :3, 0]

        # Get cocoplus or lsp joints:
        joint_x = F.matmul(verts[:, :, 0], self.joint_regressor)
        joint_y = F.matmul(verts[:, :, 1], self.joint_regressor)
        joint_z = F.matmul(verts[:, :, 2], self.joint_regressor)
        joints = F.stack([joint_x, joint_y, joint_z], axis=2)

        return verts, joints, Rs, A
