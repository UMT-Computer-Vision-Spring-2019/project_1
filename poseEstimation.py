class Camera(object):
    def __init__(self):
        self.p = None                   # Pose
        self.f = None                   # Focal Length in Pixels
        self.c = np.array([None,None])  #

    def projective_transform(self,x):
        pixels = []
        image = argv[0]
        transformed = []

        FOCAL_LENGTH = 1000
        SENSOR_X = 2000
        SENSOR_Y = 1000
        CAMERA_X = SENSOR_X/2
        CAMERA_Y = SENSOR_Y/2
        with open(image) as file:
            for p in file:
                    p = p.strip()
                    p = p.split(" ")
                    x = float(p[0])/float(p[2])
                    y = float(p[1])/float(p[2])

                    u = FOCAL_LENGTH * x + CAMERA_X
                    v = FOCAL_LENGTH * y + CAMERA_Y


                    if (u > 0 and u < 2000) and (v > 0 and v < 1000):
                        pixels.append([u,v])
                pass

    def rotational_transform(self,X):
        CAMERA_AZIMUTH = 45
        cosAz= np.cos(CAMERA_AZIMUTH)
        sinAz= np.sin(CAMERA_AZIMUTH)

        CAMERA_PITCH = -10
        cosPch= np.cos(CAMERA_PITCH)
        sinPch= np.sin(CAMERA_PITCH)
        CAMERA_ROLL = 0
        cosRoll= np.cos(CAMERA_ROLL)
        sinRoll= np.sin(CAMERA_ROLL)
        CAMERA_EASTING = 10000
        CAMERA_NORTHING = 5000
        CAMERA_ELEVATION = 1000

        T = [
        [1, 0, 0,-CAMERA_EASTING],
        [0, 1, 0,-CAMERA_NORTHING],
        [0, 0, 1,-CAMERA_ELEVATION],
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
        #C = Raxis, Rroll, Rpitch, Ryaw, T
        xList = []
        yList = []
        with open(inFile) as file:
            for p in file:
                    p = p.strip()
                    p = p.split(" ")
                    x = float(p[0])
                    y = float(p[1])
                    z = float(p[2])
                    point = [x,y,z,1]

                    translated = np.matmul(T,point)
                    yawed = np.matmul(Ryaw,translated)
                    pitched = np.matmul(Rpitch,yawed)
                    rolled = np.matmul(Rroll,pitched)
                    swapped = np.matmul(Raxis,rolled)

                    xList.append(swapped[0])
                    yList.append(swapped[1])
                pass

    def estimate_pose(self,X_gcp,u_gcp):
        """
        This function adjusts the pose vector such that the difference between the observed pixel coordinates u_gcp
        and the projected pixels coordinates of X_gcp is minimized.
        """
        pass
