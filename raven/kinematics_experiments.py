from sympy import *



def rotation(angle, axis):
    if axis == 'x':
        return Matrix([
            [1, 0,          0,          0],
            [0,  cos(angle), sin(angle), 0],
            [0, -sin(angle), cos(angle), 0],
            [0,          0,          0, 1]])
    if axis == 'y':
        return Matrix([
            [cos(angle), 0, -sin(angle), 0],
            [0,          1,         0,   0],
            [sin(angle), 0, cos(angle), 0],
            [0,          0,          0, 1]])
    if axis == 'z':
        return Matrix([
            [cos(angle), sin(angle), 0, 0],
            [-sin(angle), cos(angle), 0, 0],
            [0,          0,          1, 0],
            [0,          0,          0, 1]])

def translation(disp, axis):
    out = eye(4)
    ax_ind = {"x":0, "y":1, "z":2}[axis]
    out[ax_ind, 3] = disp
    return out

# LOWER ARM

# todo: use dh

var("theta6 alpha theta5 slide_lower")

R6inv = rotation(theta6,'z')
T6inv = translation(alpha, 'x')
R5inv = rotation(theta5,'y')
T5inv = translation(slide_lower, 'x')
M_upper = R6inv * T6inv * R5inv * T5inv


# UPPER ARM

var("phi1 beta phi2 gamma")
R1 = rotation(phi1, 'z')
R12 =  rotation(beta, 'x')
R2 = rotation(phi2, 'z')
R3 = rotation(gamma,'x')
M_lower =  R1 * R12 * R2 * R3


# total

R6 = rotation(-theta6,'z')
T6 = translation(-alpha, 'x')
R5 = rotation(-theta5,'y')
T4 = translation(-slide_lower, 'x')

var("slide_fixed")




M_total = R1 * R12 * R2 * R3 * translation(slide_fixed,'z') * T4 * R5 * T6 * R6

# Given an arbitrary normalized vector, should be able to solve M_lower[:3,2] = [px, py, pz]
# Actually that's easy