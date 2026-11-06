"""Pure command admission; never changes actuator gains or protection limits."""
from __future__ import annotations
import math
import numpy as np

class CommandSlewLimiter:
    def __init__(self,linear_acceleration:float,yaw_acceleration:float):
        if not all(math.isfinite(x) and x>0 for x in (linear_acceleration,yaw_acceleration)):
            raise ValueError('command acceleration limits must be finite and positive')
        self.linear_acceleration=float(linear_acceleration)
        self.yaw_acceleration=float(yaw_acceleration)
        self.command=np.zeros(3,dtype=np.float64)

    def step(self,requested,dt:float):
        goal=np.asarray(requested,dtype=np.float64)
        if goal.shape!=(3,) or not np.isfinite(goal).all() or not math.isfinite(dt) or dt<=0:
            raise ValueError('finite3command and positive timestep required')
        delta=goal-self.command;length=float(np.linalg.norm(delta[:2]))
        maximum=self.linear_acceleration*dt
        if length>maximum:delta[:2]*=maximum/length
        delta[2]=np.clip(delta[2],-self.yaw_acceleration*dt,self.yaw_acceleration*dt)
        self.command+=delta
        return self.command.copy()
