import torch
from torch import Tensor
from typing import Optional
import copy

""" Gravitational acceleration in m.s-2 """
GRAVITY = 9.80665

class WheelParams:
    """
    Parameters of a single wheel

    Attributes:
        radius - radius of the wheel
        ro - distance of IMU unit from the centre
        xyz - coordinates of the wheel centrum w.r.t. robot's origin
        steerable - boolean flag
        side - L/R
    """
    def __init__(self, radius:float=0.1, xyz:tuple[float,float,float]=(0., 0., 0.), steerable:bool=False, side:str='L', ro:float=0.0):
        self.radius = torch.scalar_tensor(radius)
        self.ro = torch.scalar_tensor(ro)
        self.xyz = torch.tensor(xyz)
        self.steerable = steerable
        self.side = side

def body2world_matrix(rpy: Tensor) -> Tensor:
    """ Returns transformation matrix between body and world frame """
    sx, cx = torch.sin(rpy[0]),  torch.cos(rpy[0])
    sy, cy = torch.sin(rpy[1]),  torch.cos(rpy[1])
    sz, cz = torch.sin(rpy[2]),  torch.cos(rpy[2])

    return torch.stack((
        torch.stack((cy*cz,     sx*sy*cz-cx*sz,     cx*sy*cz+sx*sz), dim=-1),
        torch.stack((cy*sz,     sx*sy*sz+cx*cz,     cx*sy*sz-sx*cz), dim=-1),
        torch.stack((  -sy,              sx*cy,              cx*cy), dim=-1)
    ), dim=-2)

def wheel2body_matrix(wheelAngles: Tensor, steerAngles: Tensor, sgn: Tensor) -> Tensor:
    """ Returns transformation matrices between wheels and body. Sgn: 1 = left, -1 = right """

    # calculate transformation from wheel to world coordinates
    s1, c1 = torch.sin(steerAngles), torch.cos(steerAngles)
    # s2, c2 = torch.sin(sgn*torch.pi/2), torch.cos(sgn*torch.pi/2)
    s3, c3 = torch.sin(sgn*wheelAngles), torch.cos(sgn*wheelAngles)

    # R = Rz(steering) * Rx(sgn*pi/2) * Rz(sgn*wheel)
    z = torch.zeros(sgn.shape)
    return torch.stack((
        torch.stack((c1*c3,  -c1*s3,  s1*sgn), dim=-1),
        torch.stack((c3*s1,  -s1*s3, -c1*sgn), dim=-1),
        torch.stack((sgn*s3, sgn*c3,       z), dim=-1)
    ), dim=-2)


class RobotState:
    """
    Status of the robot model

    Attributes:
        linVelocity - linear velocity of the body in body coordinates (x', y', z')
        angVelocity - angular velocity of the body in body coordinates (x', y', z')
        xyz - cartesian global coordinates in NED (x, y, z)
        rpy - Z-Y-X Euler angles
        wheelSpeeds - angular velocity of wheel rotation
        wheelAngles - angle of the wheel rotation
        steerSpeeds - steering rates
        steerAngles - steering angles
    """
    def __init__(self, numWheels:int=4, data:Optional[Tensor]=None) -> None:
        if data != None:
            self.linVelocity = data[0:3]
            self.angVelocity = data[3:6]
            self.xyz = data[6:9]
            self.rpy = data[9:12]
            self.wheelSpeeds = data[12 : 12 + numWheels]
            self.wheelAngles = data[12 + numWheels : 12 + 2*numWheels]
            self.steerSpeeds = data[12 + 2*numWheels : 12 + 3*numWheels]
            self.steerAngles = data[12 + 3*numWheels : 12 + 4*numWheels]
        else:
            self.linVelocity = torch.zeros(3, requires_grad=True)
            self.angVelocity = torch.zeros(3, requires_grad=True)
            self.xyz = torch.zeros(3, requires_grad=True)
            self.rpy = torch.zeros(3, requires_grad=True)
            self.wheelSpeeds = torch.zeros(numWheels, requires_grad=True)
            self.wheelAngles = torch.zeros(numWheels, requires_grad=True)
            self.steerSpeeds = torch.zeros(numWheels, requires_grad=True)
            self.steerAngles = torch.zeros(numWheels, requires_grad=True)

    def copy(self):
        """ returns shallow copy of the object """
        return copy.copy(self)

    def to_tensor(self) -> Tensor:
        """ converts RobotState to torch.Tensor """
        return torch.cat((
            self.linVelocity,
            self.angVelocity,
            self.xyz,
            self.rpy,
            self.wheelSpeeds,
            self.wheelAngles,
            self.steerSpeeds,
            self.steerAngles
        ))
    
    def wheel2body(self, sgn: Tensor) -> Tensor:
        """ Returns array of 2D tensors (as 3D tensor) """
        return wheel2body_matrix(self.wheelAngles, self.steerAngles, sgn)
    
    def body2world(self) -> Tensor:
        """ Returns rotation between the robot and world frame """
        return body2world_matrix(self.rpy)
    
    @staticmethod
    def size(numWheels:int) -> int:
        """ Count of elements in state """
        return 12 + 4*numWheels
    
    @staticmethod
    def add(x1:Tensor, x2:Tensor) -> Tensor:
        """Sums two RobotState-s as tensors into one, considering angles."""
        tmp = x1 + x2
        numWheels = (tmp.shape[0] - 12) // 4
        y = RobotState(numWheels, data=tmp)
        y.rpy = angle(y.rpy)
        y.steerAngles = angle(y.steerAngles)
        return y.to_tensor()

def angle(x: Tensor) -> Tensor:
    """ converts angle to range -pi to pi """
    return x - 2*torch.pi * torch.round(x / (2*torch.pi))

class SensorData:
    """
    Class represents all sensors of the robot

    Attributes:
        body - body IMU data (wx, wy, wz, ax, ay, az)
        wheels - list of wheel IMU data
    """
    def __init__(self,  numWheels:int=4, data:Optional[Tensor]=None) -> None:
        if data != None:
            self.body = data[0:6]
            self.wheels = [data[6+6*k : 12+6*k] for k in range(numWheels)]
        else:
            self.body = torch.zeros(6)
            self.wheels = [torch.zeros(6) for _ in range(numWheels)]
           
    def to_tensor(self) -> Tensor:
        """ convert SensorData to torch.Tensor """
        return torch.cat((
            self.body,
            *self.wheels
        ))
    
    @staticmethod
    def size(numWheels: int) -> int:
        return 6 + 6*numWheels