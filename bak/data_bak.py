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
FAN_VELOCITY = 3 # m/s
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
	return [ dist_acclr*np.cos(phi_body), dist_acclr * np.sin(phi_body) ]

def max_drag_force_on_body(Cd, row, v, A):
	v_mag = (v[0] ** 2 + v[1] ** 2) ** 0.5
	max_drag = (Cd * (0.5) * row * (v_mag ** 2) * A)
	#max_drag_x = (Cd * (0.5) * row * (v[0] ** 2) * A)
	#max_drag_y = (Cd * (0.5) * row * (v[1] ** 2) * A)
	#return [ max_drag_x, max_drag_y ] 

	return max_drag 

def drag_acclr_on_body(Cd, row, v_body, A, m, phi_body):
	max_drag = max_drag_force_on_body(Cd=Cd, row=row, v=v_body, A=A)
	dist_drag_acclr = (max_drag / m) * (np.cos(phi_body)) # [1, 0, 1]% drag depending on phi
	return dist_drag_acclr

def acclr(Cd, row, v_body, A, m, phi_body):
	max_drag = max_drag_force_on_body(Cd=Cd, row=row, v=v_body, A=A)
	max_fan = max_pressure_force_from_fan(row=row, v=FAN_VELOCITY, A=A)

	#drag_acclr = max_drag
	drag_acclr = [ (max_drag * np.cos(phi_body) / m), (max_drag * np.sin(phi_body) / m) ]
	fan_acclr = (max_fan / m) * np.cos(phi_body)


	acclr_x = (fan_acclr * np.cos(phi_body) - drag_acclr[0])
	acclr_y = (fan_acclr * np.sin(phi_body) - drag_acclr[1]) - g
	acclr = [ acclr_x, acclr_y ]
	#body_acclr = acclr_from_fan(row=row, v=FAN_VELOCITY, A=A, m=m, phi_body=phi_body)
	#drag_acclr = drag_acclr_on_body(Cd=Cd, row=row, v_body=v_body, A=A, m=m, phi_body=phi_body)
	#print('%s %s' % (body_acclr, drag_acclr))
	return acclr

def velocity(vi, t, A, m, phi_body, Cd, row):
	a = acclr(Cd=Cd, row=row, v_body=vi, A=A, m=m, phi_body=phi_body)
	v_x = vi[0] + a[0] * t
	v_y = vi[1] + a[1] * t
	# if v_y < -20:
	# 	v_y = -20
	v = [ v_x, v_y ]
	return v

def position(t_step, t_cur, xi, yi, vi, A, m, phi_body, Cd, row):
	v = velocity(vi=vi, t=t_step, A=A, m=m, phi_body=phi_body, Cd=Cd, row=row)
	#print(v)
	x = xi + (v[0] * t_step)
	y = yi + (v[1] * t_step)
	return [ x, y, v[0], v[1] ] 

t_cur = 0
t_max = 0.5
t_step = 0.0005

fig = plot.figure(1)
ax = plot.axes(projection='3d')


'''
	Paper
'''
correction_constant = int(t_max/t_step) / 100
theta_pool = np.linspace(-np.pi/2, np.pi/2, 10)
for t in theta_pool:
	n = 0
	phi_cur = t # modify later
	pos_cache_ = np.zeros((int(t_max/t_step), 4), dtype=np.float64)
	pos_cache_[0,0] = p0[0]
	pos_cache_[0,1] = p0[1]
	pos_cache_[0,2] = v0[0]
	pos_cache_[0,3] = v0[1]

	t_cur = t_step
	while t_cur < t_max:
		n += 1
		x_l = pos_cache_[n-1,0]
		y_l = pos_cache_[n-1,1]
		v_i = [ pos_cache_[n-1,2], pos_cache_[n-1,3] ]
		#print("%s %s %s %s" % (n, x_l, y_l, v_i))

		cur_pos = position(t_step=t_step, t_cur=t_cur, xi=x_l, yi=y_l,
		 vi=v_i, A=SURFACE_AREA[0], m=MASS[0], phi_body=phi_cur,
		  Cd=c_drag_orthogonal, row=air_density)

		# minor oscillation
		if phi_cur > 0:
			phi_cur = phi_cur - (1 / correction_constant) * (phi_cur)
		elif phi_cur < 0:
			phi_cur = phi_cur + (1 / correction_constant) * np.abs(phi_cur)

		pos_cache_[n,0] = cur_pos[0]
		pos_cache_[n,1] = cur_pos[1]
		pos_cache_[n,2] = cur_pos[2]
		pos_cache_[n,3] = cur_pos[3]

		t_cur = t_cur + t_step

		if(cur_pos[1] < 0):
			t_cur = t_max

	#pp.pprint(pos_cache_)

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




