from sympy import symbols
from sympy.physics.mechanics import *


# #####-------------------------------
# symbols
# ----------------------------------------
# norm sim
parra_deploy_t, touch_down_t, t_shut, t = symbols('p_t g_t t_sh t')
m, eng_m, g, max_mot, max_thrust, max_impulse = symbols('m eng_m g max_mot max_thrust max_impulse')
# min max angle

# dim symbols, unit vects
pos_x, pos_y, pos_z = dynamicsymbols('x y z')  # normal vector location
theta_x, theta_y, theta_z = dynamicsymbols('theta_x theta_y theta_z')
eng_x, eng_y, eng_z = dynamicsymbols('eng_x eng_y eng_z')


# eng_t = dynamicsymbols('t_eng')  # magnitude
f, force_para = dynamicsymbols('f f_p')
# sum_f = dynamicsymbols('F')
mot_tx = dynamicsymbols('mot_t')  # variable step
mot_ty = dynamicsymbols('mot_t')

# -------------------------------------------
# frame setup
# ----------------------------------------

# reference frames
N = ReferenceFrame('N')
Ro = ReferenceFrame('Ro')  # rocket refernce, thrust always at cent, maybe ad mom later
engine_ref = ReferenceFrame('engine_ref')

# points
pad = Point('pad')
COM = Point('COM')
engine_point = Point('engine_point')
nose_point = Point('nose_point')
rocket_geo_cent = Point('ro_cent')
tail_point = Point('tail_point')

# -----
# extra setup
# --------

# constant
comst = []