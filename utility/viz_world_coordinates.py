import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from scipy.spatial.transform import Rotation as R

arrow_prop_dict = dict(mutation_scale=10, arrowstyle='->', shrinkA=0, shrinkB=0)

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)

        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


class CoordAxis:

    def __init__(self, ax, length = 1):
        self.length = length
        self.ax = ax
        self.x = Arrow3D([0.5, 0.5+length], [0.0, 0.0], [0, 0], **arrow_prop_dict, color='r')
        self.y = Arrow3D([0.5, length], [0, length], [0, 0], **arrow_prop_dict, color='b')
        self.z = Arrow3D([0.5, length], [0, 0], [0, length], **arrow_prop_dict, color='g')

    def plot(self):
        for axis in [self.x, self.y, self.z]:
            self.ax.add_artist(axis)


    def update(self, X, Y):
        '''
        :param X: coordinates on X axis
        :param Y: coordinates on Y axis
        :return: update internal representation
        '''
        # X, Y must have length 3
        assert len(X) == 3 and len(Y) == 3

        XY = np.array(list(zip(X, Y)))
        xx, yy, zz = list(zip(X, X))

        self.x = Arrow3D(XY[0], yy, zz, **arrow_prop_dict, color='r')
        self.y = Arrow3D(xx, XY[1], zz, **arrow_prop_dict, color='b')
        self.z = Arrow3D(xx, yy, XY[2], **arrow_prop_dict, color='g')

    def transform(self, pose = None):
        '''
        :param pose: 6 DOF parameters [x, y, z, roll, pitch, yaw]
        :return: transform coordinate axis
        '''
        # compute orientation matrix
        euler = np.array(pose[3:])
        # convert euler to rotation matrix
        rot_mat = R.from_euler('xyz', euler, degrees=False).as_matrix()
        # extract position vector from pose
        X = np.array(pose[:3])
        # arrow length is fixed and depends on the origin
        Y = X + np.array([self.length, self.length, self.length])
        # don't forget to update  coordinates!
        XY = np.array([X, Y]).T

        XY = rot_mat @ XY

        self.update(XY[:, 0], XY[:, 1])

    def clear(self):
        self.ax.cla()




if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    pose = [0.5, 0.25, 0.1, 0.0, 0, 0]
    ref = CoordAxis(ax, length=1)
    ref.transform(pose)
    ref.plot()
    # axis limits
    ax.set_xlim3d(-5, 5)
    ax.set_ylim3d(-5, 5)
    ax.set_zlim3d(0, 5)
    # display
    plt.show()