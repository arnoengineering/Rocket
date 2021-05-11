

def cost_f(pos0, pos1):
    pass


def get_pos():
    pass


def get_pos_expected():
    pass


def motor_turn():
    # p0 = get_pos()
    # p1 = get_pos_expected()
    # cost_f(p0, p1)
    pass
# f = Piecewise((-f, t <= t_shut), (0, True)) * engine_ref.z  # hofuly this works
# f_mot = (f, engine_point)
# f_p_f = Piecewise((-fpr * fp, parra_deploy_t < t), (0, True))
# hofuly this works, maybe just loop and solve for curr dt
# f_para = (f_p_f*Ro.z, nose_point)

# # point shpere
# com_sp = Sphere(color='yellow', radius=0.125)
# eng_sp = Sphere(color='black', radius=0.1)
# nose_sp = Sphere(color='black', radius=0.1)
#
# # point vis
# com_p = vis_f(COM, com_sp)
# eng_p = vis_f(engine_point, eng_sp)
# nose_p = vis_f(nose_point, nose_sp)

# # basic diff
# eq_ex = [alpha_mag - w_mag.diff(t), alpha_mot_mag - w_mot_mag.diff(t),  a_mag - v_mag.diff(t)]
# # sum_f - m*a
# diff_eq = [w_mag - theta.diff(t), w_mot_mag - theta_mot.diff(t), v_mag - pos_mag.diff(t)]
# from pydy.viz.shapes import Cylinder, Sphere
# from pydy.viz.scene import Scene

# from sympy.physics.vector import *