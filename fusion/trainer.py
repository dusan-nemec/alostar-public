import torch
from torch.nn import Module
from structures import RobotState
from collections import deque 
from typing import Optional

class Trainer:
    """ Class implements training of robot parameters """
    def __init__(self, model: Module) -> None:
        self.model = model
        self.numParams = sum(1 for _ in model.parameters())
        self.opt = torch.optim.Adam(model.parameters(), lr=5e-6)
        self.buffLen = 100
        self.model.train()
        self.cnt = 0

    def process(self, corr: RobotState):
        err = corr.to_tensor()
        loss = (err ** 2).sum()
        loss.backward()

        self.cnt += 1
        if self.cnt >= self.buffLen:
            self.opt.step()
            self.model.zero_grad()
            self.cnt = 0
