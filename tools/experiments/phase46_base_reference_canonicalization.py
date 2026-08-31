"""Exact P(base_control_frame) <-> M(base_body origin) diagnostic transforms."""
from __future__ import annotations
import numpy as np

def skew(x):
 x=np.asarray(x,float);return np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])
def transforms(rotation_world_from_base,offset_m_to_p_base):
 r=np.asarray(rotation_world_from_base)@np.asarray(offset_m_to_p_base);x_pm=np.eye(16);x_pm[:3,3:6]=-skew(r);return x_pm,np.linalg.inv(x_pm),r
def xdot(rotation_world_from_base,offset_m_to_p_base,omega_world):
 r=np.asarray(rotation_world_from_base)@np.asarray(offset_m_to_p_base);d=np.zeros((16,16));d[:3,3:6]=-skew(np.cross(omega_world,r));return d
def configuration_m_to_p(position_m,rotation,joints,offset):return np.asarray(position_m)+np.asarray(rotation)@np.asarray(offset),np.asarray(rotation).copy(),np.asarray(joints).copy()
def configuration_p_to_m(position_p,rotation,joints,offset):return np.asarray(position_p)-np.asarray(rotation)@np.asarray(offset),np.asarray(rotation).copy(),np.asarray(joints).copy()
def velocity_m_to_p(nu_m,x_pm):return np.asarray(x_pm)@np.asarray(nu_m)
def velocity_p_to_m(nu_p,x_mp):return np.asarray(x_mp)@np.asarray(nu_p)
def acceleration_m_to_p(a_m,nu_m,x_pm,x_dot):return np.asarray(x_pm)@np.asarray(a_m)+np.asarray(x_dot)@np.asarray(nu_m)
def acceleration_p_to_m(a_p,nu_m,x_mp,x_dot):return np.asarray(x_mp)@(np.asarray(a_p)-np.asarray(x_dot)@np.asarray(nu_m))
def force_p_to_m(q_p,x_pm):return np.asarray(x_pm).T@np.asarray(q_p)
def force_m_to_p(q_m,x_mp):return np.asarray(x_mp).T@np.asarray(q_m)
def mass_p_to_m(m_p,x_pm):return np.asarray(x_pm).T@np.asarray(m_p)@np.asarray(x_pm)
def bias_p_to_m(h_p,m_p,nu_m,x_pm,x_dot):return np.asarray(x_pm).T@(np.asarray(h_p)+np.asarray(m_p)@np.asarray(x_dot)@np.asarray(nu_m))
def jacobian_p_to_m(j_p,x_pm):return np.asarray(j_p)@np.asarray(x_pm)
def jdotv_p_to_m(b_p,j_p,nu_m,x_dot):return np.asarray(b_p)+np.asarray(j_p)@np.asarray(x_dot)@np.asarray(nu_m)
def reduction_p_to_m(n_p,c_p,nu_m,x_mp,x_dot):return np.asarray(x_mp)@np.asarray(n_p),np.asarray(x_mp)@(np.asarray(c_p)-np.asarray(x_dot)@np.asarray(nu_m))
def observable_p_to_m(o_p,b_p,nu_m,x_pm,x_dot):return np.asarray(o_p)@np.asarray(x_pm),np.asarray(b_p)+np.asarray(o_p)@np.asarray(x_dot)@np.asarray(nu_m)
