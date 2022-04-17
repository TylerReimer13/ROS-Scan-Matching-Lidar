import numpy as np
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt


class ICP:
    def __init__(self):
        pass

    @staticmethod
    def rotation_matrix(ang, direction="cw"):
        d = 1
        if direction == "cw":
            d = -1
        elif direction == "ccw":
            d = 1
        else:
            print("Rotation direction not valid. Needs to be 'cw' or 'ccw'")

        r = np.array([[np.cos(ang), -d * np.sin(ang)],
                      [d * np.sin(ang), np.cos(ang)]])

        return r

    def test_data(self, rotation=0., translation=np.zeros(2)):
        # Only used for generating dummy test data
        ds1 = np.array([[0., 0.],
                        [0., 2.],
                        [1., 3.],
                        [2., 2.],
                        [2., 0.]])

        r = self.rotation_matrix(rotation)

        ds2 = ds1 + translation
        ds2_rot = []
        for xy in ds2:
            new_pt = r @ xy
            ds2_rot.append(new_pt)
        ds2_rot = np.array(ds2_rot)

        return ds1, ds2_rot

    def nearest_neighbor(self, pc1, pc2):
        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(pc2)
        _, indices = neigh.kneighbors(pc1)

        return indices.ravel()

    def nearest_neighbor_SLOW(self, pc1, pc2):
        idxs = []
        for pt1 in pc1:
            nearest = 9E99
            nearest_idx = None
            pt2_idx = 0
            for pt2 in pc2:
                dist = np.linalg.norm(pt1 - pt2)
                if dist < nearest:
                    nearest = dist
                    nearest_idx = pt2_idx

                pt2_idx += 1

            idxs.append(nearest_idx)

        return idxs

    def centroid(self, pts):
        length = pts.shape[0]
        sum_x = np.sum(pts[:, 0])
        sum_y = np.sum(pts[:, 1])
        return np.array([sum_x / length, sum_y / length])

    def transformation(self, pc1, pc2):
        cent_pc1 = self.centroid(pc1)
        P = pc1 - cent_pc1

        cent_pc2 = self.centroid(pc2)
        Q = pc2 - cent_pc2

        M = np.dot(P.T, Q)  # Covariance

        U, W, V_t = np.linalg.svd(M)
        R = np.dot(V_t.T, U.T)  # Rotation

        t = cent_pc2 - np.dot(R, cent_pc1.T)  # Translation

        T = np.identity(3)  # Full transformation matrix (includes rotation and translation)
        T[:2, 2] = np.squeeze(t)
        T[:2, :2] = R

        return T

    def transform_pc(self, pc, T):
        new_pts = []

        for pt in pc:
            # Apply rotation and translation to each point in point cloud
            new_pt = (T[:2, :2] @ pt) + T[:2, 2]
            new_pts.append(new_pt)

        return np.array(new_pts)

    def __call__(self, pc1, pc2, n_iter=1):
        A = pc1.copy()
        B = pc2.copy()

        for i in range(n_iter):
            neighbors = self.nearest_neighbor(A, B)
            T = self.transformation(A, B[neighbors])  # T = self.transformation(pc1, targets)
            A = self.transform_pc(A, T)

        T = self.transformation(pc1, A)
        return T, pc2



