import torch
from torch import Tensor
from torch.nn import Module, Parameter
from structures import WheelParams

class Kinematics(Module):
    """
    Converts wheel angular velocities to body velocity vector
    """
    def __init__(self, wheelParams: list[WheelParams]) -> None:
        super().__init__()
        self.numWheels = len(wheelParams)
        self.whRadius = Parameter(torch.stack([w.radius for w in wheelParams]))
        self.whPosition = torch.stack([w.xyz for w in wheelParams])
        self.whSteerable = torch.tensor([1. if w.steerable else 0. for w in wheelParams])

    def forward(self, wheelSpeeds: Tensor, steerAngles: Tensor) -> tuple[Tensor, Tensor]:
        """ 
        Computes the body velocity in local coordinates 

        Arguments:
            wheelSpeeds - tensor of wheel angular velocities (+ forward / - backward)
            steerAngles - tensor of steering angles (+ right / - left)
        
        Returns:
            tuple of body linear velocity [vx, vy, vz], body angular velocity [wx, wy, wz]
        """
        # TODO adaptive weightening by skid probability
        #W = torch.zeros(2*self.numWheels, requires_grad=True) # wheel velocities
        #D = torch.zeros(2*self.numWheels, 3, requires_grad=True) # W = D * (vx, vy, wz)
        #for k in range(self.numWheels):
        #    W[2*k] = wheelSpeeds[k] * self.whRadius[k] * torch.cos(state.steerAngles[k])
        #    W[2*k+1] = wheelSpeeds[k] * self.whRadius[k] * torch.sin(state.steerAngles[k])
        #    D[2*k, 0] = 1.0
        #    D[2*k, 2] = -self.whPosition[1] # yi
        #    D[2*k+1, 1] = 1.0
        #    D[2*k+1, 2] = self.whPosition[0] # xi
        W = torch.cat((
            wheelSpeeds * self.whRadius * torch.cos(steerAngles),
            wheelSpeeds * self.whRadius * torch.sin(steerAngles)
        ))

        e = torch.ones(self.numWheels)
        n = torch.zeros(self.numWheels)

        D = torch.cat((
            torch.stack((e, n, -self.whPosition[:,1]), dim=1),
            torch.stack((n, e,  self.whPosition[:,0]), dim=1),
        ))

        x = torch.linalg.lstsq(D, W).solution
        v = torch.tensor((1., 1., 0.)) * x
        w = torch.tensor((0., 0., 1.)) * x
        return v, w


    def inverse(self, velocity: Tensor, angVelocity: Tensor) -> tuple[Tensor, Tensor]:
        """ 
        Computes the wheel speed and steering angles from given velocity vectors

        Arguments:
            velocity - linear velocity vector in body frame
            angVelocity - angular velocity vector in body frame
        
        Returns:
            tuple of wheel speeds and steering angles (zero for non-steerable wheels)
        """
        #W = torch.zeros(2*self.numWheels, requires_grad=True) # wheel velocities
        #D = torch.zeros(2*self.numWheels, 3, requires_grad=True) # W = D * (vx, vy, wz)
        #for k in range(self.numWheels):
        #    W[2*k] = wheelSpeeds[k] * self.whRadius[k] * torch.cos(state.steerAngles[k])
        #    W[2*k+1] = wheelSpeeds[k] * self.whRadius[k] * torch.sin(state.steerAngles[k])
        #    D[2*k, 0] = 1.0
        #    D[2*k, 2] = -self.whPosition[1] # yi
        #    D[2*k+1, 1] = 1.0
        #    D[2*k+1, 2] = self.whPosition[0] # xi
        
        N = self.numWheels
        e = torch.ones(N)
        n = torch.zeros(N)

        D = torch.cat((
            torch.stack((e, n, -self.whPosition[:,1]), dim=1),
            torch.stack((n, e,  self.whPosition[:,0]), dim=1),
        ))

        vw = torch.stack((
            velocity[0],
            velocity[1],
            angVelocity[2]
        ))

        W = D @ vw
        wR_cos = W[0:N]
        wR_sin = W[N:2*N]
        steerAngles = torch.atan2(wR_sin, wR_cos) * self.whSteerable

        wheelSpeeds = (wR_cos * torch.cos(steerAngles) + wR_sin * torch.sin(steerAngles)) / self.whRadius
        return wheelSpeeds, steerAngles