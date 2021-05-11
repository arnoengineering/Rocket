import numpy as np
from sympy import sin, cos
from pydy.viz.visualization_frame import VisualizationFrame
from RocketObjects import *
from RocketSymbols import *
from scipy.integrate import odeint
from pydy.codegen.ode_function_generators import generate_ode_function
# from matplotlib import pyplot as plt

# let r = radius, T = torque, I=mass inertia, m = mass, f = force(mag), F = force(vect)
# notes
"""refernce frames dont have any positions, only rotations, point can move in these frames, 
asuming x is stationary relitive to F, it might move through F; but since f nas no positions, x is the def of F so any 
points in F mut move with angle f And vel x in order to maintain relation, : """
# init_vprinting(use_latex='mathjax', pretty_print=False)


def gen_sym(q_val):
    if not q_val.is_Derivative:
        der_sym = dynamicsymbols('v_'+q_val.name)
        # if 'theta' in q_val.name:
        #
        kd.append(der_sym - q_val.diff(t))
        val_speeds.append(der_sym)
    pass


def com_con(ls, out_ls, ref):
    for val in ls:  # todo funct
        if type(val) == Vector:
            for va in components(val, ref):
                gen_sym(va)
                out_ls.append(va)  # todo change ref
        else:
            out_ls.append(val)


def components(fun, ref_frame):
    # fr = m
    return np.array(list(fun.to_matrix(ref_frame)))


def vector_base(ve):
    return ve.x, ve.y, ve.z


def set_pos_ro(p, p_l):
    p.set_pos(tail_point, np.dot(p_l, vector_base(Ro)))


def vis_f(poi, sh):
    return VisualizationFrame(Ro, poi, sh)


def two_ref_frame(tor):
    return (engine_ref, tor), (Ro, -tor)


# const
g_a = -9.81

# ####------------------------
# classes
# ---------------
rocket_obj = Rocket()
engine_obj = Engine('D10-5')
Motor_obj = Motor()

# set vectors
theta = theta_x*N.x + theta_y*N.y + theta_z*N.z
theta_mot = eng_x * Ro.x + eng_y * Ro.y + eng_z * Ro.z
pos = pos_x*N.x + pos_y*N.y + pos_z*N.z

# vector magitudes

theta_norm = theta.normalize()
theta_mot_norm = theta_mot.normalize()
theta_m = theta.magnitude()
theta_mot_m = theta_mot.magnitude()

# set Frames
Ro.orient_quaternion(N, (cos(theta_m/2), *components(theta_norm, N)*sin(theta_m/2)))
engine_ref.orient_quaternion(Ro, (cos(theta_mot_m/2), *components(theta_mot_norm, Ro)*sin(theta_mot_m/2)))

v = pos.diff(t, N)  # needed?
a = v.diff(t, N)
w = theta.diff(t, N)
w_mot = theta_mot.diff(t, Ro)
alpha = w.diff(t, N)
alpha_mot = w_mot.diff(t, Ro)

# velocities
Ro.set_ang_vel(N, w)
engine_ref.set_ang_vel(Ro, w_mot)

# set points
set_pos_ro(COM, rocket_obj.COM)
set_pos_ro(engine_point, engine_obj.pos)
set_pos_ro(nose_point, rocket_obj.nose)
# set_pos_ro(rocket_geo_cent, np.array(para_loc) / 2)

# point relations
COM.set_vel(Ro, 0*Ro.x)
COM.set_vel(N, v)

engine_point.v2pt_theory(COM, N, Ro)
nose_point.v2pt_theory(COM, N, Ro)

# inertias
rocket_I = (inertia(Ro, *rocket_obj.inertia), COM)
engine_I = (inertia(Ro, *rocket_obj.inertia), COM)

# bodies
rocket = RigidBody('Rocket', COM, Ro, m, rocket_I)
engine = RigidBody('Engine', engine_point, engine_ref, m, engine_I)
bodys = [rocket, engine]

# forces
eng_F = -f * engine_ref.z
eng_T = cross(engine_point.pos_from(COM), eng_F)
g_f = g*m*N.y

mot_Tx = mot_tx*engine_ref.x
mot_Ty = mot_ty*engine_ref.y

# vector tuples
eng_F_tuple = (engine_point, eng_F)
g_tuple = (COM, g_f)

# torques, ref frame
eng_T_tuple_eng, eng_T_tuple_rocket = two_ref_frame(eng_T)  # todo is needed
mot_Tx_tuple_eng, mot_Tx_tuple_rocket = two_ref_frame(mot_Tx)
mot_Ty_tuple_eng, mot_Ty_tuple_rocket = two_ref_frame(mot_Ty)  # todo add reation forces

coords = [pos, theta]
theta_co = [theta_mot]

# speed
speeds = [v, w]
theta_sp = [w_mot]

force_ls = [eng_F_tuple, g_tuple, mot_Ty_tuple_eng, mot_Ty_tuple_rocket,
            mot_Tx_tuple_eng, mot_Tx_tuple_rocket, eng_T_tuple_eng, eng_T_tuple_rocket]

val_coords = []
val_speeds = []
kd = []

com_con(coords, val_coords, N)
# com_con(speeds, val_speeds, N)
com_con(theta_co, val_coords, Ro)
# com_con(theta_sp, val_speeds, Ro)

# for q, u in zip(val_coords, val_speeds):
#     kd.append(u-q.diff(t))
    
kane = KanesMethod(N, val_coords, val_speeds, kd)
print(kane.q)
print(kane.u)
# force list


fr, frst = kane.kanes_equations(bodys, force_ls)

mass_matrix = kane.mass_matrix_full
forcing_vector = kane.forcing_full
constants = [g, f]  # todo add a constant mag multiplied by step
print(forcing_vector)
print(mass_matrix)

# ##-----------------
# solve
# peicewise: solve mag
sp = [f]
const_val = [g_a, engine_obj.force]

r_h_s = generate_ode_function(forcing_vector, val_coords, val_speeds,
                              constants, mass_matrix=mass_matrix, specifieds=sp)
# simulate
frames_per_sec = 60
final_time = 5

t_range = np.linspace(0.0, final_time, final_time * frames_per_sec)

y = odeint(r_h_s, np.array([0, 0, 0]), t_range, args=(np.array([0]), const_val))
