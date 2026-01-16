import torch
from torch import Tensor
from torch.nn import Module, Parameter

class Calibration6DoF(Module):
    """
    Calibration module for 6-DoF sensor
    """
    def __init__(self) -> None:
        super().__init__()
        self.gainDeviation = Parameter(torch.zeros(6))
        self.bias = Parameter(torch.zeros(6))
        # TODO alignment

    def forward(self, raw:Tensor) -> Tensor:
        return (raw - self.bias) * (1.0 + self.gainDeviation)