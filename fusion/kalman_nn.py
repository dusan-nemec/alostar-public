import torch
from torch import Tensor
from torch.nn import Module
from torch.autograd.functional import jacobian
from torch.func import jacrev # type: ignore
from typing import Optional
import os

class EKF(Module):
    def __init__(self, f, h, lim:Optional[Tensor]=None):
        """
        Creates new extended Kalman filter

        Arguments:
            f - state predict function x[k] = f(x[k-1], u[k])
            h - output function y[k] = h(x[k])
            lim - optional state correction limit (symmetric)
        """
        super().__init__()
        self.f = f
        self.h = h

        if os.name != 'nt':
            self.f = torch.compile(self.f)
            self.h = torch.compile(self.h)

        self.df = jacrev(self.f, argnums=(0, 1))
        self.dh = jacrev(self.h, argnums=0)

        self.lim = lim
        self.debugData = []
        self.debugAxis = -1

    def forward(self, x:Tensor, P:Tensor, u:Tensor, U:Tensor, z:Optional[Tensor]=None, R:Optional[Tensor]=None) -> tuple[Tensor, Tensor, Optional[Tensor]]:
        """
        Runs EKF estimator

        Arguments:
            x - previous state x[k-1]
            P - previous state covariance P[k-1]
            u - input tensor u[k]
            U - input tensor covariance U[k]
            z - ouptut tensor z[k]
            R - ouput tensor covariance R[k]

        Returns:
            tuple of (x, P, corr) - estimated state, its covariance and correction
        """
        # predict
        x_new = self.f(x, u)

        # df_dx, df_du = jacobian(self.f, (x.detach(), u.detach()), create_graph=False)
        with torch.no_grad():
            df_dx, df_du = self.df(x.detach(), u.detach())

        P_new = df_dx @ P @ df_dx.T + df_du @ U @ df_du.T

        if torch.isnan(x_new).any():
            raise Exception("NaN occured")
        
        if z == None or R == None:
            # skip update phase
            return x_new, P_new, None
        else:
            # update phase
            y = self.h(x_new)
            e = z - y
            e = torch.where(torch.isnan(e), torch.tensor(0.), e)
            
            # dh_dx = jacobian(self.h, x_new.detach(), create_graph=False)
            with torch.no_grad():
                dh_dx = self.dh(x_new.detach())
            S = R + dh_dx @ P_new @ dh_dx.T
            K = torch.linalg.solve(S, P_new @ dh_dx.T, left=False)
            
            if self.debugAxis >= 0:
                self.debugData.append((K[self.debugAxis,:] * e).tolist())
            corr = K @ e
            if self.lim is not None:
                corr = torch.clip(corr, -self.lim, self.lim)
            x_up = x_new + corr
            P_up = (torch.eye(x.size(0)) - K @ dh_dx) @ P_new

            if torch.isnan(x_up).any():
                raise Exception("NaN occured")
            
            return x_up, P_up, corr
        
class ESEKF(Module):
    def __init__(self, f, h, com, errSize:int, lim:Optional[Tensor]=None):
        """
        Creates new error state extended Kalman filter

        Arguments:
            f - state predict function x[k] = f(x[k-1], u[k])
            h - output function y[k] = h(x[k])
            com - composition function x_true[k] = com(x_predicted[k], x_error[k])
            errSize - size of the error state vector
            lim - optional state correction limit (symmetric)
        """
        super().__init__()
        self.f = f
        self.h = h
        self.com = com
        self.errSize = errSize
        self.lim = lim

    def forward(self, x:Tensor, P:Tensor, u:Tensor, U:Tensor, z:Optional[Tensor]=None, R:Optional[Tensor]=None) -> tuple[Tensor, Tensor, Optional[Tensor]]:
        """
        Runs EKF estimator

        Arguments:
            x - previous state x[k-1]
            P - previous error state covariance P[k-1]
            u - input tensor u[k]
            U - input tensor covariance U[k]
            z - ouptut tensor z[k]
            R - ouput tensor covariance R[k]

        Returns:
            tuple of (x, P, corr) - estimated state, error state covariance, correction error
        """
        # predict
        x_new = self.f(x, u)
        s = torch.zeros(self.errSize)
        xd = x.detach()
        ud = u.detach()
        df_dx, df_du = jacobian(self.f, (xd, ud), create_graph=False)
        _, dx_ds = jacobian(self.com, (xd, s), create_graph=False)
        df_ds = df_dx @ dx_ds
        P_new = df_ds @ P @ df_ds.T + df_du @ U @ df_du.T

        if z == None or R == None:
            # skip update phase
            return x_new, P_new, None
        else:
            # update phase
            y = self.h(x_new)
            e = z - y
            e = torch.where(torch.isnan(e), torch.tensor(0.), e)
            dh_dx = jacobian(self.h, x_new.detach(), create_graph=False)
            dh_ds = dh_dx @ dx_ds
            S = R + dh_ds @ P_new @ dh_ds.T
            K = torch.linalg.solve(S, P_new @ dh_ds.T, left=False)
            corr = K @ e
            if self.lim is not None:
                corr = torch.clip(corr, -self.lim, self.lim)
            x_up = self.com(x_new, corr)
            P_up = (torch.eye(self.errSize) - K @ dh_ds) @ P_new
            return x_up, P_up, corr