
import torch
from torch import Tensor
from torch.nn import Module, Parameter, ParameterList, ModuleList
from kalman_nn import ESEKF, EKF
from typing import Optional
from structures import SensorData, RobotState, WheelParams, GRAVITY, body2world_matrix, wheel2body_matrix, angle
from kinematics import Kinematics
from calibration import Calibration6DoF

# This is an alternative to calib_nn module, using multiple smaller EKFs
class PoseEstimator(Module):
    """
    Complete pose estimator
    """
    def __init__(self, wheelParams: list[WheelParams], calibIMU:bool=False) -> None:
        super().__init__()
        self.numWheels = len(wheelParams)
        if calibIMU:
            self.bodyCalib = Calibration6DoF()
            self.wheelCalib = ModuleList([Calibration6DoF() for _ in range(self.numWheels)])
        else:
            self.bodyCalib = None
            self.wheelCalib = None


        self.wheelParams = wheelParams
        self.whSteerable = torch.tensor([1. if w.steerable else 0. for w in wheelParams])
        self.whSgn = torch.tensor([1.0 if w.side == 'L' else -1.0 for w in wheelParams])
        self.kinematics = Kinematics(wheelParams=wheelParams)

        # body attitude estimator state = (roll, pitch, yaw)
        self.rpyEKF = EKF(f=self.rpy_predict, h=self.rpy_output, lim=torch.tensor((.1, .1, .1)))
        self.rpyCovariance = torch.eye(3)

        # wheel attitude estimator, state = (wheelSpeeds, wheelAngles, steerSpeeds, steerAngles) 
        self.wheelEKF = EKF(f=self.wheel_predict, h=self.wheel_output)
        self.wheelCovariance = torch.eye(self.numWheels * 4)

        # pose estimator, state = (linVelocity, xyz)     
        self.poseEKF = EKF(f=self.pose_predict, h=self.pose_output)
        self.poseCovariance = torch.eye(6)
        self.linAcceleration = torch.zeros(3) # debug

        # compact state structure
        self.state = RobotState(numWheels=self.numWheels)
        self.correction = RobotState(numWheels=self.numWheels)
        self.sensorRMS = SensorData(self.numWheels, torch.ones(SensorData.size(self.numWheels)))
        self.bodySensCovariance = torch.eye(6)
        self.wheelSensCovariance = torch.eye(6 * self.numWheels)
        self.dt = torch.scalar_tensor(1.0)
        self.reset_alignment()


    def reset_alignment(self):
        """
        Resets state of the alignment forcing it to be re-done
        """
        self.aligned = False
        self.alignSum = SensorData(self.numWheels)
        self.alignSum2 = SensorData(self.numWheels)
        self.alignCnt = 0
        self.alignInvalid = 0
    

    def align(self, sensors: SensorData) -> bool:
        """
        Runs initial alignment
        """
        if self.aligned:
            return True
        
        acc = sensors.body[3:6]
        if torch.linalg.vector_norm(acc) < 0.1 * GRAVITY:
            self.alignInvalid += 1
            return False
        
        for w in sensors.wheels:
            acc = w[3:6]
            if torch.linalg.vector_norm(acc) < 0.1 * GRAVITY:
                self.alignInvalid += 1
                return False
            
        # add to accumulator
        self.alignSum.body += sensors.body
        self.alignSum2.body += sensors.body ** 2
        for k in range(self.numWheels):
            self.alignSum.wheels[k] += sensors.wheels[k]
            self.alignSum2.wheels[k] += sensors.wheels[k] ** 2
        self.alignCnt += 1

        if self.alignCnt >= 100:
            n = self.alignCnt
            acc = self.alignSum.body[3:6]  # no need to normalize
            bodyRoll = torch.atan2(-acc[1], -acc[2]).float()
            bodyPitch = torch.atan2(acc[0], torch.sqrt(acc[1] ** 2 + acc[2] ** 2)).float()
            bodyYaw = self.state.rpy[2].float() # copy the yaw from initial setup
            self.state.rpy = torch.tensor((bodyRoll, bodyPitch, bodyYaw))
            self.sensorRMS.body = (torch.sqrt((self.alignSum2.body / n) - (self.alignSum.body / n) ** 2) * torch.tensor((1., 1., 1., 10., 10., 10.))).detach()
            self.bodySensCovariance = torch.diag(self.sensorRMS.body ** 2).detach()

            wheelAngles = []
            for s, s2, p, k in zip(self.alignSum.wheels, self.alignSum2.wheels, self.wheelParams, range(self.numWheels)):
                acc = s[3:6]
                sgn = 1.0 if p.side == 'L' else -1.0
                fi = sgn * torch.atan2(-sgn*acc[0], -sgn*acc[1]).float() + bodyPitch
                wheelAngles.append(fi)
                self.sensorRMS.wheels[k] = torch.sqrt((s2 / n) - (s / n) ** 2).detach()
            self.state.wheelAngles = torch.tensor(wheelAngles)
            self.wheelSensCovariance = torch.diag(torch.cat(self.sensorRMS.wheels) ** 2).detach()

            print(f"\nInvalid samples: {self.alignInvalid}")

            print(f"\nAverage values:")
            print(f"body: {self.alignSum.body / n}")
            for k, w in enumerate(self.alignSum.wheels):
                print(f"wheel #{k+1}: {w/n}")

            print(f"\nEstimated IMU RMS:")
            print(f"body: {self.sensorRMS.body}")
            for k, w in enumerate(self.sensorRMS.wheels):
                print(f"wheel #{k+1}: {w}")

            # calculate biases of wheel sensors
            if self.wheelCalib != None and self.bodyCalib != None:
                wheel2body = wheel2body_matrix(self.state.wheelAngles, self.state.steerAngles, self.whSgn)
                body2world = body2world_matrix(self.state.rpy)
                body_acc = body2world.T @ self.gravity()

                s = self.alignSum.body
                gyro_bias = s[0:3] / n
                acc_bias = s[3:6] / n - body_acc
                bias = torch.cat((gyro_bias, acc_bias)).detach()
                self.bodyCalib.bias = Parameter(bias)
                
                for k, s in enumerate(self.alignSum.wheels):
                    gyro_bias = s[0:3] / n
                    acc_bias = s[3:6] / n - wheel2body[k].T @ body_acc
                    bias = torch.cat((gyro_bias, acc_bias)).detach()
                    self.wheelCalib[k].bias = Parameter(bias)

                print(f"\nEstimated IMU bias:")
                print(f"body: {self.bodyCalib.bias}")
                for k, w in enumerate(self.wheelCalib):
                    print(f"wheel #{k+1}: {w.bias}")
            
            self.aligned = True
            return True
        else:
            return False
        
    def odometry(self, state: RobotState, sensors: SensorData, dt: Tensor) -> RobotState:
        """ Runs baseline odometry for comparison"""
        if not self.aligned:
            self.align(sensors)
            return state
        
        # calibrate
        if self.bodyCalib != None:
            sensors.body = self.bodyCalib.forward(sensors.body)

        if self.wheelCalib != None:
            sensors.wheels = [wc.forward(x) for x, wc in zip(sensors.wheels, self.wheelCalib)]

        new = RobotState(self.numWheels)
        new.wheelSpeeds = torch.tensor([wd[2] for wd in sensors.wheels]) * self.whSgn
        new.wheelAngles = state.wheelAngles + new.wheelSpeeds * dt
        
        new.linVelocity, new.angVelocity = self.kinematics.forward(new.wheelSpeeds, new.steerAngles)
        yaw = state.rpy[2] + new.angVelocity[2] * dt
        new.rpy = torch.stack((state.rpy[0], state.rpy[1], yaw))

        new.xyz = state.xyz + new.body2world() @ new.linVelocity * dt
        return new


    def forward(self, state: RobotState, sensors: SensorData, dt: Tensor) -> tuple[RobotState, RobotState]:
        """
        Runs robot localization model 

        Arguments:
            state - state of the estimator
            sensors - sensor readings
            dt - sampling period

        Returns:
            new state and correction error 
        """
        self.state = RobotState(self.numWheels, state.to_tensor().detach())
        self.dt = dt

        if not self.aligned:
            # initial estimate of the wheel and body attitude from accelerometer readings
            self.align(sensors)
            return self.state, self.correction
        
        # calibrate
        if self.bodyCalib != None:
            sensors.body = self.bodyCalib.forward(sensors.body)

        if self.wheelCalib != None:
            sensors.wheels = [wc.forward(x) for x, wc in zip(sensors.wheels, self.wheelCalib)]

        # AHRS
        self.state.rpy, self.rpyCovariance, rpyCorrection = self.rpyEKF.forward(
            x=self.state.rpy, 
            P=self.rpyCovariance, 
            u=sensors.body,
            U=self.bodySensCovariance,
            z=sensors.body,
            R=self.bodySensCovariance
        )

        # save state, copy body angular rate
        self.state.angVelocity = sensors.body[0:3]

        if rpyCorrection != None:
            self.correction.rpy = rpyCorrection

        # wheels
        wheelState = torch.cat((self.state.wheelSpeeds, self.state.wheelAngles, self.state.steerSpeeds, self.state.steerAngles))
        wheelData = torch.cat(sensors.wheels)
        wheelState, self.wheelCovariance, wheelCorrection = self.wheelEKF.forward(
            x=wheelState,
            P=self.wheelCovariance,
            u=wheelData,
            U=self.wheelSensCovariance,
            z=wheelData,
            R=self.wheelSensCovariance
        )
        N = self.numWheels
        self.state.wheelSpeeds = wheelState[0:N]
        self.state.wheelAngles = wheelState[N:2*N]
        self.state.steerSpeeds = wheelState[2*N:3*N]
        self.state.steerAngles = wheelState[3*N:4*N]

        if wheelCorrection != None:
            self.correction.wheelSpeeds = wheelCorrection[0:N]
            self.correction.wheelAngles = wheelCorrection[N:2*N]
            self.correction.steerSpeeds = wheelCorrection[2*N:3*N]
            self.correction.steerAngles = wheelCorrection[3*N:4*N]

        # velocity and pose
        poseState = torch.cat((self.state.linVelocity, self.state.xyz))
        sens = sensors.to_tensor()
        sensCovariance = torch.block_diag(self.bodySensCovariance, self.wheelSensCovariance)
        poseState, self.poseCovariance, poseCorrection = self.poseEKF.forward(
            x=poseState,
            P=self.poseCovariance,
            u=sens,
            U=sensCovariance,
            z=sens,
            R=sensCovariance
        )

        self.state.linVelocity = poseState[0:3]
        self.state.xyz = poseState[3:6]
        
        if poseCorrection != None:
            self.correction.linVelocity = poseCorrection[0:3]
            self.correction.xyz = poseCorrection[3:6]

        return self.state, self.correction


    def rpy_predict(self, rpy: Tensor, u: Tensor) -> Tensor:
        """ Predicts new Euler angles from body sensor measurements """
        sx, cx = torch.sin(rpy[0]), torch.cos(rpy[0])
        sy, cy = torch.sin(rpy[1]), torch.cos(rpy[1])
        e, n = torch.scalar_tensor(1.), torch.scalar_tensor(0.)

        gyro2euler = torch.stack((
            torch.stack(( e, sx*sy/cy,  cx*sy/cy), dim=-1),
            torch.stack(( n,       cx,       -sx), dim=-1),
            torch.stack(( n,    sx/cy,     cx/cy), dim=-1)
        ), dim=-2)

        return rpy + gyro2euler @ u[0:3] * self.dt

    def rpy_output(self, rpy: Tensor) -> Tensor:
        """ Returns expected body acceleration """
        world2body = body2world_matrix(rpy).T
        angVelocity = self.state.angVelocity.detach()
        linVelocity = self.state.linVelocity.detach()

        gyro = torch.full((3,), float('nan'))
        acc =  ( 
            world2body @ self.gravity() # gravity
            + torch.cross(angVelocity, linVelocity, dim=0) # radial acceleration
            # + self.linAcceleration.detach()
        )
        return torch.cat((gyro, acc))
    
    def wheel_predict(self, s: Tensor, u: Tensor) -> Tensor:
        """ Predicts new state (wheelSpeeds, wheelAngles, steerSpeeds, steerAngles) from sensor measurement"""
        n = self.numWheels
        oldWheelSpeeds = s[0:n]
        oldWheelAngles = s[n:2*n]
        oldSteerSpeeds = s[2*n:3*n]
        oldSteerAngles = s[3*n:4*n]
        wheelsData = u.reshape(n, 6)

        # extract wheel speed from IMUs
        wheelSpeeds = wheelsData[:,2] * self.whSgn # wz
        # TODO low-pass filter

        # update rotation of wheels
        #wheelAngles = oldWheelAngles + (oldWheelSpeeds + wheelSpeeds) * (self.dt/2)
        wheelAngles = oldWheelAngles + wheelSpeeds * self.dt

        # extract steer speed from IMU, see wheel2body matrix
        s3, c3 = torch.sin(self.whSgn*wheelAngles), torch.cos(self.whSgn*wheelAngles)

        steerSpeeds = self.whSteerable * (self.whSgn*s3*wheelsData[:,0] + self.whSgn*c3*wheelsData[:,1] - self.state.angVelocity[2].detach())
        # TODO low-pass filter

        # update steer angles
        #steerAngles = oldSteerAngles + (oldSteerSpeeds + steerSpeeds) * (self.dt/2)
        steerAngles = oldSteerAngles + steerSpeeds * self.dt
        steerAngles = self.whSteerable * angle(steerAngles)
        #steerAngles = torch.clip(steerAngles, torch.scalar_tensor(-1.0), torch.scalar_tensor(1.0))
        return torch.cat((wheelSpeeds, wheelAngles, steerSpeeds, steerAngles))

    def wheel_output(self, s: Tensor) -> Tensor:
        """ Predicts wheel sensors measurements """
        # predict the body acceleration from pose, gravity and trajectory, copy the body angular velocity
        n = self.numWheels
        wheelSpeeds = s[0:n]
        wheelAngles = s[n:2*n]
        #steerSpeeds = s[2*n:3*n]
        steerAngles = s[3*n:4*n]
        world2body = self.state.body2world().T.detach()
        angVelocity = self.state.angVelocity.detach()
        linVelocity = self.state.linVelocity.detach()
        body_acc = ( 
            world2body @ self.gravity() # gravity
            + torch.cross(angVelocity, linVelocity, dim=0) # radial acceleration
        )
        
        wheel2body = wheel2body_matrix(wheelAngles, steerAngles, self.whSgn)
        z = torch.scalar_tensor(0.)

        sensorData = []

        for k in range(n):
            params = self.wheelParams[k]
            # predict the wheel acceleration and angular rate
            hubAcc = (
                body_acc
                + torch.cross(angVelocity, torch.cross(angVelocity, params.xyz, dim=0), dim=0) # centrifugal acceleration from robot's body rotation
            )
            acc = (
                wheel2body[k].T @ hubAcc.detach() # sensitive only to wheel attitude
                + torch.stack((- params.ro * (wheelSpeeds[k] ** 2), z, z)) # centrifugal acceleration from wheel own rotation
            )

            gyro = torch.full((3,), float('nan'))
            sensorData.append(torch.cat((gyro, acc)))
        return torch.cat(sensorData)


    def pose_predict(self, s: Tensor, u: Tensor) -> Tensor:
        """ Predicts new state (linVelocity, xyz) from all sensors measurement """
        sensors = SensorData(self.numWheels, u)
        oldLinVelocity = s[0:3]
        oldXYZ = s[3:6]

        # convert measured acceleration to body frame, 
        # subtract ro * wd.z**2 radial 
        # subtract radial acceleration w.r.t. body w x (w x ri)
        wheel2body = self.state.wheel2body(self.whSgn)
        angVelocity = self.state.angVelocity

        accels = [
            dcm @ (wd[3:6] + torch.tensor((params.ro*wd[2]**2, 0., 0.)))
                - torch.cross(angVelocity, torch.cross(angVelocity, params.xyz, dim=0), dim=0)
            for wd, dcm, params in zip(sensors.wheels, wheel2body, self.wheelParams)
        ]

        # calculate mean of accelerations, rotate to world frame
        accels = torch.stack(accels, dim=-1)
        body2world = self.state.body2world()

        # world-frame approach
        # acc_with_gravity = body2world @ accels.mean(dim=1) # naiive average needs to be replaced by weighted sum

        # # subtract gravity
        # self.linAcceleration = acc_with_gravity - self.gravity()

        # # update velocity
        # linVelocity = oldLinVelocity + self.linAcceleration * self.dt

        # # enforce forward velocity
        # # fwdVect = body2world @ torch.tensor((1., 0., 0.))
        # # linVelocity = fwdVect * torch.dot(fwdVect, linVelocity)

        # # update position
        # # xyz = oldXYZ + (oldLinVelocity + linVelocity) * (self.dt/2)
        # xyz = oldXYZ + linVelocity * self.dt

        # body-frame approach
        # calculate forward acceleration, zero-out the other components
        body_gravity = body2world.T @ self.gravity()
        self.linAcceleration = (accels.mean(dim=1) - body_gravity) * torch.tensor((1., 0., 0.)) # only body-forward
        linVelocity = oldLinVelocity + self.linAcceleration * self.dt

        # update position
        xyz = oldXYZ + body2world @ linVelocity * self.dt
        return torch.cat((linVelocity, xyz))

    def pose_output(self, s: Tensor) -> Tensor:
        """ Returns the expected sensor data from pose state """
        linVelocity = s[0:3]
        # xyz = s[3:6]
        angVelocity = self.state.angVelocity
        
        sensors = SensorData(self.numWheels)

        # body2world = self.state.body2world().detach()
        # body_acc = ( 
        #     body2world.T @ self.gravity() # gravity
        #     + torch.cross(angVelocity, linVelocity, dim=0).detach() # radial acceleration
        # )
        sensors.body = torch.full((6,), float('nan')) #torch.cat((angVelocity, body_acc))
        wheel2body = self.state.wheel2body(self.whSgn).detach()
        z = torch.scalar_tensor(0.)
        
        # predict the wheel angular velocity using inverse kinematics
        wheelSpeeds, _ = self.kinematics.inverse(linVelocity, angVelocity)
        
        for k in range(self.numWheels):
            hubGyro = (angVelocity + torch.stack((z, z, self.state.steerSpeeds[k]))).detach()
            gyro = (
                wheel2body[k].T @ hubGyro # body rotation + steering
                + torch.stack((z, z, wheelSpeeds[k] * self.whSgn[k])) # wheel rotation
            ) 
            acc = torch.full((3,), float('nan'))
            sensors.wheels[k] = torch.cat((gyro, acc))
        return sensors.to_tensor()

    def gravity(self) -> Tensor:
        """ Returns gravity vector in world coordinates. """
        return torch.tensor((0., 0., -GRAVITY))
