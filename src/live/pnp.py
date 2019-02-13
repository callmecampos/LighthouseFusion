import numpy as np

class Poser:
    def __init__(self, *args):
        self.diode_array = [loc for loc in args]

    def get_pose(self, *args):
        '''
        Triangulate 6-DoF pose of 4 diode 1 lighthouse system given
        azimuth and elevation angles of each diode.
        '''
        xy_n = [(np.tan(u), np.tan(v)) for u, v in args]
        A = np.zeros((self.num_diodes()*2, 8))
        b = np.array(xy_n).ravel()
        for r in range(0, self.num_diodes()*2, 2):
            i = r / 2
            A[r] = [self.x(i), self.y(i), 1, 0, 0, 0, \
                        -self.x(i)*xy_n[i][0], -self.y(i)*xy_n[i][0]]
            A[r+1] = [0, 0, 0, self.x(i), self.y(i), 1, \
                        -self.x(i)*xy_n[i][1], -self.y(i)*xy_n[i][1]]

        print(A.shape)
        print(b.shape)

        try:
            _h = np.linalg.solve(A, b)
        except np.linalg.linalg.LinAlgError:
            _h = scipy.optimize.least_squares(A, b, method="lm") # FIXME:
        # H = _h.append(1).reshape((3, 3))
        s = Poser.recover_scale(_h)

        x, y, z = s*_h[2], s*_h[5], -s
        R_mat = Poser.compute_rotation(_h)

        return (x, y, z, R_mat)

    def num_diodes(self):
        return len(self.diode_array)

    def x(self, i):
        return self.diode_array[i][0]

    def y(self, i):
        return self.diode_array[i][1]

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
