
from typing import Any
import numpy as np

gravity = 9.80665

class IMU:
    """ Class simulates wheel-mounted inertial unit (3-axial gyroscope + 3-axial accelerometer).
    The orientation of the axes w.r.t. wheel are: x = radial, y = tangential, z = axial
    """

    def __init__(self, 
            offs=(0.0, 0.0, 0.0),
            sampleRate=100.0,
            side='L',
            gyroMax=np.deg2rad(200.0),
            accMax=2.0*gravity,
            gyroResolution=16,
            accResolution=16,
            gyroBias=(0.0, 0.0, 0.0), 
            accBias=(0.0, 0.0, 0.0), 
            gyroGain=(1.0, 1.0, 1.0),
            accGain=(1.0, 1.0, 1.0),
            gyroRMS=0.0,
            accRMS=0.0) -> None:
        """ Initializes IMU object.
        
        Args:
            offs - position of the IMU w.r.t. wheel center (x, y, z) in meters

            sampleRate - rate of data in Hz

            side - side of the wheel, R = right, L = left

            gyroMax - maximal angular velocity in rad.s-1

            accMax - maximal acceleration in m.s-2

            gyroResolution - resolution of the gyroscope A/D in bits (None means analog)

            accResolution - resolution of the accelerometer A/D in bits (none means analog) 

            gyroBias - gyroscope offset (x, y, z) in rad.s-1

            accBias - accelerometer offset (x, y, z) in m.s-2

            gyroGain - gyroscope gain (x, y, z)

            accGain - accelerometer gain (x, y, z)

            gyroRMS - RMS of gyroscope white noise (all axes) in rad.s-1

            accRMS - RMS of the accelerometer white noise (all axes) in m.s-2
            
        """
        self.offs = offs
        self.sampleRate = sampleRate
        self.gyroMax = gyroMax
        self.accMax = accMax
        self.gyroResolution = gyroResolution
        self.accResolution = accResolution
        self.gyroBias = gyroBias
        self.accBias = accBias
        self.gyroGain = gyroGain
        self.accGain = accGain
        self.gyroRMS = gyroRMS
        self.accRMS = accRMS
        self.wheelAngle = 0.0
        self.steerAngle = 0.0
        self.lastSample = None
        self.lastGyro = None
        self.lastAcc = None
        self.side = side
        self.rng = np.random.default_rng()

    def sample(self, 
        t, 
        wheelAngVelocity, 
        wheelAngle=None,
        robotAccel=(0.0, 0.0, 0.0),
        robotAngVelocity=(0.0, 0.0, 0.0),
        robotRotMatrix=np.eye(3)
    ) -> tuple:
        """ Reads new sample from IMU unit.
       
        Args:
            t - timestamp of the sample in seconds, e.g. time.time()

            wheelAngVelocity - angular velocity of the wheel in rad.s-1

            wheelAngle - when set, overrides simulated wheel angle

            robotAccel - linear acceleration (x, y, z) of the robot in m.s-2

            robotAngVelocity - angular velocity (x, y, z) of the robot chasis in rad.s-1

            robotRotMatrix - column-wise rotation matrix transforming from global coordinates to robot's
        
        Returns:
            tuple: (gyro, accel) readings
        """
        if self.lastSample is None:
            dt = 0.0
        else:
            dt = t - self.lastSample
            if self.sampleRate > 0.0 and dt < (1.0 / self.sampleRate - 1e-6):
                return (self.lastGyro, self.lastAcc)

        self.lastSample = t

        # wheel angular velocity in rad.s-1
        w = np.array((0.0, 0.0, wheelAngVelocity)) 

        # update wheel angle 
        if wheelAngle is not None:
            self.wheelAngle = wheelAngle
        else:
            self.wheelAngle += wheelAngVelocity * dt

        # rotation of the wheel with respect to robot
        # first apply steering angle around z-axis
        c, s = np.cos(self.steerAngle), np.sin(self.steerAngle)
        wheelRotMatrix = np.array([
            (  c,    s,  0.0),
            ( -s,    c,  0.0),
            (0.0,  0.0,  1.0)
        ])

        # then rotate 90 degrees around x-axis (-90 for right wheels)
        sgn = (1.0 if self.side == 'L' else -1.0)
        wheelRotMatrix = np.array([
            (1.0,  0.0,  0.0),
            (0.0,  0.0,  sgn),
            (0.0, -sgn,  0.0)
        ]) @ wheelRotMatrix
        
        #then apply wheel rotation around z-axis
        c, s = np.cos(self.wheelAngle), np.sin(self.wheelAngle)
        wheelRotMatrix = np.array([
            (  c,    s,  0.0),
            ( -s,    c,  0.0),
            (0.0,  0.0,  1.0)
        ]) @ wheelRotMatrix

        # add robot's own rotation
        w = w + np.array(robotAngVelocity) @ wheelRotMatrix.T

        # transform gravity + robot's own acceleration
        aRobot = robotAccel + np.array((0, 0, -gravity)) @ robotRotMatrix.T
        a = aRobot @ wheelRotMatrix.T

        # add centrifugal acceleration
        a = a + np.array(self.offs) * (wheelAngVelocity ** 2)

        # add noise if set
        if self.gyroRMS > 0.0:
            w = w + self.rng.normal(scale=self.gyroRMS, size=3)
        if self.accRMS > 0.0:
            a = a + self.rng.normal(scale=self.accRMS, size=3)
        
        # truncate and discretize
        if self.gyroMax is not None:
            w = w.clip(-self.gyroMax, self.gyroMax)
            if self.gyroResolution is not None:
                w = ((w / self.gyroMax) * 2 ** (self.gyroResolution-1)).astype(np.int32)
        
        if self.accMax is not None:
            a = a.clip(-self.accMax, self.accMax)
            if self.accResolution is not None:
                a = ((a / self.accMax) * 2 ** (self.accResolution-1)).astype(np.int32)
        
        self.lastGyro = w
        self.lastAcc = a
        return (w, a)