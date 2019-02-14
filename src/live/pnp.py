import numpy as np
from math import sin, cos, tan

class Pose:
    def __init__(self, x, y, z, quat=None, euler=None, R_mat=None):
        assert not (quat and euler) # FIXME
        self._x = x
        self._y = y
        self._z = z
        self.quat, self.euler = None, None
        if quat:
            self.quat = quat
        elif euler:
            self.euler = euler
        self.R = R_mat

    def x(self):
        return self._x

    def y(self):
        return self._y

    def z(self):
        return self._z

    def theta_x(self):
        if self.euler:
            return self.euler[0]

    def theta_y(self):
        if self.euler:
            return self.euler[1]

    def theta_z(self):
        if self.euler:
            return self.euler[2]

    # TODO: fill in quaternion methods

class Poser:
    def __init__(self, *args):
        self.diode_array = [loc for loc in args]

    def get_pose(self, *args):
        '''
        Triangulate 6-DoF pose of 4 diode 1 lighthouse system given
        azimuth and elevation angles of each diode.
        '''
        xy_n = [(tan(u), tan(v)) for u, v in args] # optimize for numpy array?
        A = np.zeros((self.num_diodes()*2, 8))
        b = np.array(xy_n).ravel()
        for r in range(0, self.num_diodes()*2, 2):
            i = int(r / 2)
            A[r] = [self.diode_x(i), self.diode_y(i), 1, 0, 0, 0, \
                        -self.diode_x(i)*xy_n[i][0], -self.diode_y(i)*xy_n[i][0]]
            A[r+1] = [0, 0, 0, self.diode_x(i), self.diode_y(i), 1, \
                        -self.diode_x(i)*xy_n[i][1], -self.diode_y(i)*xy_n[i][1]]

        try:
            # _h = np.linalg.solve(A, b)
            _h = np.linalg.lstsq(A, b)[0]

            s = Poser.recover_scale(_h)
            x, y, z = s*_h[2], s*_h[5], -s
            R_mat = Poser.compute_rotation(_h)

            return Pose(x, y, z, R_mat=R_mat)
        except np.linalg.linalg.LinAlgError:
            estimator = LMOptimization(self, xy_n)
            estimator.compute_pose()
            return estimator.pose()

    def num_diodes(self):
        return len(self.diode_array)

    def diode_x(self, i):
        return self.diode(i)[0]

    def diode_y(self, i):
        return self.diode(i)[1]

    def diode(self, i):
        return self.diode_array[i]

    @staticmethod
    def recover_scale(h):
        return 2.0 / (np.sqrt(h[0]**2 + h[3]**2 + h[6]**2) + \
                        np.sqrt(h[2]**2 + h[4]**2 + h[7]**2))

    @staticmethod
    def compute_rotation(_h):
        h1, h2, h3, h4, h5, h6, h7, h8 = _h
        norm_factor = np.sqrt(h1**2 + h4**2 + h7**2)
        r11, r21, r31 = h1 / norm_factor, h4 / norm_factor, -h7 / norm_factor

        scale_factor = r11*h2 + r21*h5 - r31*h8
        _r2 = np.array([
                h2 - r11*scale_factor,
                h5 - r21*scale_factor,
                -h8 - r31*scale_factor
            ])

        r1 = np.array([r11, r21, r31])
        r2 = _r2 / np.linalg.norm(_r2)

        print(r1, r2)
        r3 = np.cross(r1, r2)

        return np.column_stack((r1, r2, r3))

class LMOptimization:
    def __init__(self, poser, angles,
                    p0=Pose(1, 1, 1, euler=(0, 0, 0)),
                    lda=1): # FIXME: play with best seed and lambda
        assert poser.num_diodes() == 4 # TODO: make optimization work for n > 4 lol
        self.poser = poser
        self.angles = angles
        self.p = p0
        self.lda = lda

    def pose(self):
        return self.p

    def compute_pose(self, max_iter=100):
        for _ in range(max_iter):
            self.step()

    def step(self):
        f = self.evaluate_objective()
        J_g = self.compute_jacobian_g()
        J_f = self.compute_jacobian_f()

        print(J_g)
        print(J_f)

        J = np.dot(J_f, J_g)

        print(J)

        JTJ = np.dot(J.T, J)

        self.p += np.dot( np.linalg.inv(JTJ + self.lda*np.diagonal(JTJ)), b - f )

    def evaluate_objective(self):
        result = 0
        h1, h2, h3, h4, h5, h6, h7, h8, h9 = self.g()
        for i in range(self.poser.num_diodes()):
            xn_i, yn_i = self.angles[i]
            x_i, y_i = self.poser.diode_x(i), self.poser.diode_y(i)

            f0 = (h1*x_i + h2*y_i + h3) / (h7*x_i + h8*y_i + h9)
            f1 = (h4*x_i + h5*y_i + h6) / (h7*x_i + h8*y_i + h9)

            result += (xn_i - f0)**2 + (yn_i - f1)**2
        return result

    def compute_jacobian_g(self):
        if self.p.euler:
            x, y, z = self.p.euler
            return np.array([
                [-cos(x)*sin(y)*sin(z), -sin(y)*cos(z)-sin(x)*cos(y)*sin(z),
                    -cos(y)*sin(z)-sin(x)*sin(y)*cos(z), 0, 0, 0],
                [sin(x)*sin(z), 0, -cos(x)*cos(z), 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [cos(x)*sin(y)*cos(z), -sin(y)*sin(z)+sin(x)*cos(y)*cos(z),
                    -cos(y)*cos(z)-sin(x)*sin(y)*sin(z), 0, 0, 0],
                [-sin(x)*cos(z), 0, -cos(x)*sin(z), 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [-sin(x)*sin(y), cos(y)*cos(y),
                    -cos(y)*cos(z)-sin(x)*sin(y)*sin(z), 0, 0, 0],
                [-sin(x)*cos(z), 0, -cos(x)*sin(z), 0, 0, 0],
                [0, 0, 0, 0, 0, -1]
            ])
        elif self.p.quat:
            return None # FIXME: implement for quaternion representation
        else:
            return None # rotation matrix representation?

    def compute_jacobian_f(self):
        h1, h2, h3, h4, h5, h6, h7, h8, h9 = self.g()
        rows = [
            np.array([
                [x_i / (h7*x_i + h8*y_i + h9), y_i / (h7*x_i + h8*y_i + h9), 1 / (h7*x_i + h8*y_i + h9), 0, 0, 0,
                    -(h1*x_i + h2*y_i + h3) / (h7*x_i + h8*y_i + h9)**2,
                    (h1*x_i + h2*y_i + h3) / (h7*x_i + h8*y_i + h9)**2,
                    -(h1*x_i + h2*y_i + h3) / (h7*x_i + h8*y_i + h9)**2],
                [0, 0, 0, x_i / (h7*x_i + h8*y_i + h9), y_i / (h7*x_i + h8*y_i + h9), 1 / (h7*x_i + h8*y_i + h9),
                    -(h4*x_i + h4*y_i + h6) / (h7*x_i + h8*y_i + h9)**2,
                    (h4*x_i + h4*y_i + h6) / (h7*x_i + h8*y_i + h9)**2,
                    -(h4*x_i + h4*y_i + h6) / (h7*x_i + h8*y_i + h9)**2]
            ])
            for x_i, y_i in self.poser.diode_array
        ]
        return np.vstack(tuple(rows))

    def g(self):
        p = self.p # FIXME: I'm lazy lmao
        if p.euler:
            return np.array([
                cos(p.theta_y())*cos(p.theta_z()) - \
                    sin(p.theta_x())*sin(p.theta_y())*sin(p.theta_z()),
                -cos(p.theta_x())*sin(p.theta_z()),
                p.x(),
                cos(p.theta_y())*sin(p.theta_z()) + \
                    sin(p.theta_x())*sin(p.theta_y())*cos(p.theta_z()),
                cos(p.theta_x())*cos(p.theta_z()),
                p.y(),
                cos(p.theta_x())*sin(p.theta_y()),
                -sin(p.theta_x()),
                -p.z()
            ])
        elif self.p.quat:
            return None # FIXME: implement for quaternion representation
        else:
            return None # rotation matrix representation?
