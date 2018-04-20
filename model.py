import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import math
import collections
import pprint

'''
	Standard Units
		-- Distance: meters
		-- Weight: kilograms
		-- Force: kilograms per meter^2
'''

'''
	STATE VARIABLES:
'''
gravity = 9.81 # gravity in m/s^2
MASS = 5.20 / 1000 #0.7474 / 1000 # mass of material in grams * 1000 
SURFACE_AREA = 0.01 # m^2
INFLUENCE_TIME = 0.125 # acceptable range to assume that the orientation will not change
FAN_VELOCITY = 3 #m/s
VARIENCE = 0
THETA = 0
HEIGHT = 1 #meters
p0 = [
	0,
	HEIGHT,
]
V0 = [
	0,
	0,
]

'''
	TIME BOUNDARIES:
'''
t = 0
tmax = 2
tstep = 0.1

def y(_p0, _V0, _t):
	net_velocity_y = Vf(INFLUENCE_TIME)*np.sin(THETA)*_t - _V0[1]*_t - (0.5)*(gravity)*(_t**2)
	y = _p0[1] + net_velocity_y 
	return y, net_velocity_y

def x(_p0, _V0, _t):
	net_velocity_x = Vf(INFLUENCE_TIME)*np.cos(THETA)*_t
	x = _p0[0] + net_velocity_x + _V0[0]*_t
	return x, net_velocity_x

def Vf(_T):
	Vf = Af(MASS)*_T
	return Vf
	

def Af(_m): #magnitude of a vector
	Af = ((VARIENCE)*Fmax())/_m
	return Af

def Fmax(): #magnitude of pressure vector (force)
	air_density = 1.225 #kg/m^3
	max_pressure_force = (air_density * FAN_VELOCITY**2 * SURFACE_AREA)
	return max_pressure_force

_x = 0
_y = HEIGHT

x_max = 1
x_step = 1

x_cache = {}
y_cache = {}
xv_cache = {}
yv_cache = {}
final_velocity_cache = {}
xi_cache = {}
yi_cache = {}
time_cache = []

while p0[0] <= x_max:
	while VARIENCE <= 1:
		if(t > tmax):
			xv_cache[VARIENCE] = xi_cache
			yv_cache[VARIENCE] = yi_cache
			VARIENCE += 0.5
			VARIENCE = float("{0:.2f}".format(VARIENCE))
			t = 0
			xi_cache = {}
			yi_cache = {}
		if(VARIENCE == 0):
			time_cache.append(t)
		_x, _v_x = x(p0, V0, t)
		_y, _v_y = y(p0, V0, t)
		#print("t: %s, X: %s, Y: %s" % (t, _x, _y))
		if(_y < 0):
			tmax = t
			final_velocity_cache[VARIENCE] = [_v_x, _v_y]
		xi_cache[t] = _x
		yi_cache[t] = _y
		t += tstep
	x_cache[p0[0]] = xv_cache
	y_cache[p0[0]] = yv_cache
	p0[0] = p0[0] + x_step
	VARIENCE = 0

pp = pprint.PrettyPrinter(indent=4)

final_velocities = collections.OrderedDict(sorted(final_velocity_cache.items()))
pp.pprint(x_cache)
pp.pprint(y_cache)

# fig = plot.figure(1)
# ax = plot.axes(projection='3d')
# for key in x_cache.keys(): # key is p0[0] 
# 	temp_cache = x_cache[key] # variance array with static p0[0]
# 	for key2 in temp_cache.keys(): 
# 		plot_y = y_cache[key][key2]
# 		plot_x = x_cache[key][key2]
# 		print(plot_x)
# 		#ax.plot_wireframe(X=plot_x,Y=time_cache,Z=plot_y, color='0.1')
# ax.set_xlabel('X')
# ax.set_ylabel('Time')
# ax.set_zlabel('Y')
# ax.set_zlim(zmin=0, zmax=HEIGHT)

# plot.show()
exit()

