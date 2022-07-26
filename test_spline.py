import numpy as np
from cubic_spline import Spline2D
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy import interpolate

 

x =np.array ([0,0,0,0,0,5,10,15,20,25,30,35,40,45,50,55,60,60,60,60,60])

y = np.array([20,40,60,80,100,100,100,100,100,100,100,100,100,100,100,100,100,80,60,40,20])

ds = 0.5    # [m] distance of each intepolated points
sp = Spline2D(x, y)

s = np.arange(0, sp.s[-1], ds)
rx, ry, ryaw, rk = [], [], [], []
for i_s in s:
    ix, iy = sp.calc_position(i_s)
    rx.append(ix)
    ry.append(iy)
    ryaw.append(sp.calc_yaw(i_s))
    rk.append(sp.calc_curvature(i_s))

# Desired trajectory
traj = np.array([rx, ry])
plt.plot(x, y,'oc')
plt.plot(x,y,color='orange')
plt.plot(traj[0,:],traj[1,:],'r',label = 'Natural Spline')
plt.legend(['Data','Natural Spline','Constrained Spline'])
plt.xlabel("X")
plt.ylabel("Y")
plt.show()