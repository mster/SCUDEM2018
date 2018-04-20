import matplotlib.pyplot as plot
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import collections
import pprint

pp = pprint.PrettyPrinter(indent=4)

# useful constants
g = 9.81 # gravity in m/s^2
air_density = 1.225 # kg/m^3
c_drag_orthogonal = 1.28 #

# environment parameters
FAN_VELOCITY = 2 # m/s
HEIGHT = 1 # meters
THETA = 0 # completely horizontal = 0 

# randomness control
VARIENCE = 0

# assumptions
INFLUENCE_TIME = 0.125 # acceptable range to assume that the orientation will not change

# materials parameters
MASS = [
	0.7474 / 1000, # mass of paper in kilograms = grams / 1000 
	5.20 / 1000, # mass of cardboard in kilograms = grams / 1000 
]
SURFACE_AREA = [
	0.01,  # surface area of paper m^2
	0.01, # surface area of cardboard m^2
	]

# initial conditions
p0 = [
	0, 
	HEIGHT, # material will start at the set environment height
]
v0 = [
	0,
	0,
]

# functions
def max_pressure_force_from_fan(row, v, A):
	return (row * (v ** 2) * A)

def max_acclr_from_fan(row, v, A, m):
	F_max = max_pressure_force_from_fan(row=row, v=v, A=A)
	max_acclr = F_max / m
	return max_acclr

def acclr_from_fan(row, v, A, m, phi_body):
	max_acclr = max_acclr_from_fan(row=row, v=v, A=A, m=m)
	dist_acclr = max_acclr*(np.cos(phi_body)) # [1, 0, 1]% force depending on phi
	return dist_acclr

def max_drag_force_on_body(Cd, row, v, A):
	max_drag = (Cd * (0.5) * row * (v ** 2) * A)
	return max_drag

def drag_acclr_on_body(Cd, row, v_body, A, m, phi_body):
	max_drag = max_drag_force_on_body(Cd=Cd, row=row, v=v_body, A=A)
	dist_drag_acclr = (max_drag / m) * (np.cos(phi_body)) # [1, 0, 1]% drag depending on phi
	return dist_drag_acclr

def velocity(vi, t, A, m, phi_body, Cd, row):
	a = acclr(Cd=Cd, row=row, v_body=vi, A=A, m=m, phi_body=phi_body)
	v = vi + (a * t)
	return v

def acclr(Cd, row, v_body, A, m, phi_body):
	body_acclr = acclr_from_fan(row=row, v=FAN_VELOCITY, A=A, m=m, phi_body=phi_body)
	drag_acclr = drag_acclr_on_body(Cd=Cd, row=row, v_body=v_body, A=A, m=m, phi_body=phi_body)
	acclr = body_acclr - drag_acclr
	#print('%s %s' % (body_acclr, drag_acclr))
	return acclr

def position(t_step, t_cur, xi, yi, vi, A, m, phi_body, Cd, row):
	v = velocity(vi=vi, t=t_step, A=A, m=m, phi_body=phi_body, Cd=Cd, row=row)
	v_x = v * np.cos(phi_body)
	v_y = v * np.sin(phi_body)
	x = xi + (v * v_x * t_step)
	y = yi + (v * v_y * t_step) - (g * t_cur) * t_cur * 0.5
	v = (v_x**2 + v_y**2)*(0.5)
	return [ x, y, v ]

t_cur = 0
t_max = 1
t_step = 0.01

fig = plot.figure(1)
ax = plot.axes(projection='3d')


'''
	Paper
'''
correction_constant = 10
#theta = [-pi/2, -pi/3, -pi/4, 0, pi/4, pi/3, pi/2]
for t in np.linspace(-np.pi/2, np.pi/2, 20):
	n = 0
	phi_cur = t # modify later
	pos_cache_ = np.zeros((int(t_max/t_step), 3), dtype=np.float64)
	print(pos_cache_.shape)
	pos_cache_[0,0] = p0[0]
	pos_cache_[0,1] = p0[1]
	pos_cache_[0,2] = (v0[0]** 2 + v0[1] ** 2)

	t_cur = t_step
	while t_cur < t_max:
		n = int(t_cur/t_step)
		print(n)
		x_l = pos_cache_[n-1,0]
		y_l = pos_cache_[n-1,1]
		v_l = pos_cache_[n-1,2]
		#print("%s %s %s %s" % (n, x_l, y_l, v_l))
		if y_l < 0:
			t_max = t_cur

		cur_pos = position(t_step=t_step, t_cur=t_cur, xi=x_l, yi=y_l,
		 vi=v_l, A=SURFACE_AREA[0], m=MASS[0], phi_body=phi_cur,
		  Cd=c_drag_orthogonal, row=air_density)

		# minor oscillation
		if phi_cur is not 0:
			phi_cur = phi_cur - (1 / correction_constant) * phi_cur

		pos_cache_[n,0] = cur_pos[0]
		pos_cache_[n,1] = cur_pos[1]
		pos_cache_[n,2] = cur_pos[2]

		t_cur = t_cur + t_step

	pp.pprint(pos_cache_)

	time = np.arange(int(t_max/t_step))
	time = np.dot(time,t_step)
	x_plot = pos_cache_[:,0][:len(time)]
	y_plot = pos_cache_[:,1][:len(time)]
	ax.plot_wireframe(X=x_plot,Y=time,Z=y_plot, color='0.1')
	t_cur = 0

ax.set_xlabel('X')
ax.set_ylabel('Time')
ax.set_zlabel('Y')
ax.set_zlim(zmin=0, zmax=HEIGHT)


plot.show()
exit()




