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
FAN_VELOCITY = 1 # m/s
HEIGHT = 0.5 # meters
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

t_cur = 0
t_max = 1
t_step = 0.001
correction_constant = 50

# functions
def max_pressure_force_from_fan(row, v, A):
	return (row * (v ** 2) * A)

def dp(row, v, Cd, phi_body):
	max_dp = Cd * row * (v**2) * (0.5)
	pressure_differential = max_dp * np.cos(phi_body)
	return pressure_differential

def force_from_fan(row, v, Cd, phi_body, A):
	dp_fan = dp(row=row, v=v, Cd=Cd, phi_body=phi_body)
	F_x = dp_fan * A * np.cos(phi_body)
	F_y = dp_fan * A * np.sin(phi_body)
	return [ F_x, F_y ]

def force_from_drag(row, v, Cd, phi_body, A):
	dp_drag_x = dp(row=row, v=v[0], Cd=Cd, phi_body=phi_body)
	dp_drag_y = dp(row=row, v=v[1], Cd=Cd, phi_body=phi_body)

	F_x = dp_drag_x * A * np.cos(phi_body) - dp_drag_y * A * np.cos(phi_body)
	F_y = dp_drag_y * A * np.sin(phi_body) + dp_drag_x * A * np.sin(phi_body)
	return [ F_x, F_y ]

def acclr(Cd, row, v_body, A, m, phi_body):
	fan_force_components = force_from_fan(row=row, v=FAN_VELOCITY, Cd=Cd, phi_body=phi_body, A=A)
	drag_force_components = force_from_drag(row=row, v=v_body, Cd=Cd, phi_body=phi_body, A=A)
	F_x_net = fan_force_components[0] - drag_force_components[0]
	F_y_net = fan_force_components[1] - drag_force_components[1]


	acclr_x = (F_x_net / m)
	acclr_y = (F_y_net / m) - g
	acclr = [ acclr_x, acclr_y ]
	print(acclr)
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

fig = plot.figure(1)
ax = plot.axes(projection='3d')


'''
	Paper
'''
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
		if phi_cur != 0.0:
			phi_cur = phi_cur - (1 / correction_constant) * (phi_cur)

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




