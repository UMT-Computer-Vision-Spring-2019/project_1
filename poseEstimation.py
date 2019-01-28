from scipy.optimize import least_squares as ls
import numpy as np
import sys


class Camera(object):
    def __init__(self):
        self.p = None                   # Pose
        self.f = None                   # Focal Length in Pixels
        self.sensorX = None
        self.sensorY = None
        self.x = None
        self.y = None
        self.c = np.array([None,None])  #

    def projective_transform(self, p0, p1, p2):
        x = p0/p2
        y = p1/p2

        u = self.f * x + self.x
        v = self.f * y + self.y

        return [u,v]

    def rotational_transform(self, x):

        cosAz= np.cos(self.p[3])
        sinAz= np.sin(self.p[3])

        cosPch= np.cos(self.p[4])
        sinPch= np.sin(self.p[4])

        cosRoll= np.cos(self.p[5])
        sinRoll= np.sin(self.p[5])


        T = [
        [1, 0, 0,-self.p[0]],
        [0, 1, 0,-self.p[1]],
        [0, 0, 1,-self.p[2]],
        [0, 0, 0, 1]
        ]

        Ryaw = [
        [cosAz, -sinAz, 0, 0],
        [sinAz, cosAz, 0, 0],
        [0, 0, 1, 0]
        ]

        Rpitch = [
        [1, 0, 0],
        [0, cosPch, sinPch],
        [0, -sinPch, cosPch]
        ]

        Rroll = [
        [cosRoll, 0, -sinRoll],
        [0, 1, 0 ],
        [sinRoll, 0 , cosRoll]
        ]

        Raxis = [
        [1, 0, 0],
        [0, 0, -1],
        [0, 1 , 0]
        ]
        point = [x[0],x[1],x[2],1]

        translated = np.matmul(T,point)
        yawed = np.matmul(Ryaw,translated)
        pitched = np.matmul(Rpitch,yawed)
        rolled = np.matmul(Rroll,pitched)
        swapped = np.matmul(Raxis,rolled)

        return self.projective_transform(swapped[0],swapped[1],swapped[2])

    def estimate_pose(self,X_gcp,u_gcp):

        for gcp in X_gcp:
            gcp = self.rotational_transform(gcp)

        p_opt = ls(self.residual, self.p, method='lm',args=(X_gcp,u_gcp))['x']
        """
        This function adjusts the pose vector such that the difference between the observed pixel coordinates u_gcp
        and the projected pixels coordinates of X_gcp is minimized.
        """
    def residual(self,p,X,U):
        self.p = p
        resid =[]
        for i,gcp in enumerate(X):
            pred = self.rotational_transform(gcp)
            resid.append(pred[0] - U[i][0])
            resid.append(pred[1] - U[i][1])

        return resid




def main(argv):

    FOCAL_LENGTH = 2448
    SENSOR_X = 3264
    SENSOR_Y = 2448
    cam = Camera()
    cam.f = FOCAL_LENGTH
    cam.sensorX = SENSOR_X
    cam.sensorY = SENSOR_Y
    cam.x = cam.sensorX/2
    cam.y = cam.sensorY/2
    cam.p = np.array([0,0,0,0,0,0])

    data = [
    "|1984|1053|272558.68|5193938.07|1015 |Main hall spire|",

    "|884 |1854|272572.34|5193981.03|982 |Large spruce|",

    "|1202|1087|273171.31|5193846.77|1182 |Bottom of left tine of M|",

    "|385 |1190|273183.35|5194045.24|1137 |Large rock outcrop on Sentinel|",

    "|2350|1442|272556.74|5193922.02|998 |Southernmost window apex on main hall|"
    ]

    obs = []
    true = []

    for GCP in data:
        GCP = GCP.strip("|")
        GCP = GCP.split("|")
        true.append([float(GCP[0]),float(GCP[1])])
        obs.append([float(GCP[2]),float(GCP[3]),float(GCP[4])])

    cam.estimate_pose(obs,true)
    
    print(cam.p)

    for GCP in obs:
        print(cam.rotational_transform(GCP))


if __name__=='__main__':
  main(sys.argv[1:])
