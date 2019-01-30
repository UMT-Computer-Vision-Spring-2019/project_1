from scipy.optimize import least_squares as ls
import numpy as np
import sys


class Camera(object):
    def __init__(self,p,f,c):
        self.p = p                   # Pose
        self.f = f                   # Focal Length in Pixels
        self.c = c

    def projective_transform(self,X):
        """
        This function performs the projective transform on generalized coordinates in the camera reference frame.
        """
        projected = np.array([])

        x = X[:,0]/X[:,2]
        y = X[:,1]/X[:,2]

        u = self.f * x + self.c[0]/2
        v = self.f * y + self.c[1]/2

        u = np.hstack(u)
        v = np.hstack(v)
        return u,v


    def rotational_transform(self, X, p):

        """
        This function performs the translation and rotation from world coordinates into generalized camera coordinates.
        """


        cosAz= np.cos(p[3])
        sinAz= np.sin(p[3])

        cosPch= np.cos(p[4])
        sinPch= np.sin(p[4])

        cosRoll= np.cos(p[5])
        sinRoll= np.sin(p[5])


        T = np.mat([
        [1, 0, 0,-p[0]],
        [0, 1, 0,-p[1]],
        [0, 0, 1,-p[2]],
        [0, 0, 0, 1]])

        Ryaw = np.mat([
        [cosAz, -sinAz, 0, 0],
        [sinAz, cosAz, 0, 0],
        [0, 0, 1, 0]])

        Rpitch = np.mat([
        [1, 0, 0],
        [0, cosPch, sinPch],
        [0, -sinPch, cosPch]])

        Rroll = np.mat([
        [cosRoll, 0, -sinRoll],
        [0, 1, 0 ],
        [sinRoll, 0 , cosRoll]])

        Raxis = np.mat([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1 , 0]])

        C = Raxis @ Rroll @ Rpitch @ Ryaw @ T

        X = X.dot(C.T)

        u,v = self.projective_transform(X)

        return u,v
    def estimate_pose(self,X_gcp,u_gcp):
        """
        This function adjusts the pose vector such that the difference between the observed pixel coordinates u_gcp
        and the projected pixels coordinates of X_gcp is minimized.
        """

        p_opt = ls(self.residual, self.p, method='lm',args=(X_gcp,u_gcp))['x']

        self.p = p_opt
        return p_opt
    def residual(self,p,X,u_gcp):

        u,v = self.rotational_transform(X,p)

        u = np.squeeze(np.asarray(u - u_gcp[:,0]))
        v = np.squeeze(np.asarray(v - u_gcp[:,1]))
        resid = np.stack((u, v), axis=-1)
        resid = resid.flatten()
        #print(resid)
        return resid



def main(argv):

    FOCAL_LENGTH = 2448
    SENSOR_X = 3264
    SENSOR_Y = 2448

    f = FOCAL_LENGTH
    c = np.array([SENSOR_X,SENSOR_Y])
    p = np.array([0,0,0,0,0,0])
    cam = Camera(p,f,c)
    p_0 = np.array([0,0,0,0,0,0])
    obs = np.array([[272558.68, 5193938.07, 1015.,1],
              [272572.34, 5193981.03, 982.,1],
              [273171.31, 5193846.77, 1182.,1],
              [273183.35, 5194045.24, 1137.,1],
              [272556.74, 5193922.02, 998.,1]])

    true = np.array([[1984., 1053.],
                      [884., 1854.],
                      [1202., 1087.],
                      [385., 1190.],
                      [2350., 1442.]])



    p_opt = cam.estimate_pose(obs,true)

    for GCP in obs:
        print(cam.rotational_transform(GCP,p_opt))


if __name__=='__main__':
  main(sys.argv[1:])
