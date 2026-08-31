#!/usr/bin/env python3
import numpy as np
import phase46_base_reference_canonicalization as c
def main():
 rng=np.random.default_rng(46);R=np.eye(3);off=np.array([-.077378152,8.1e-7,-.03227768]);X,Xi,r=c.transforms(R,off);Xd=c.xdot(R,off,np.array([.3,-.2,.1]));nu=rng.normal(size=16);a=rng.normal(size=16);q=rng.normal(size=16);M=rng.normal(size=(16,16));M=M.T@M+np.eye(16);h=rng.normal(size=16);J=rng.normal(size=(7,16));N=rng.normal(size=(16,12));cp=rng.normal(size=16);O=rng.normal(size=(4,16));bp=rng.normal(size=4)
 p=rng.normal(size=3);joints=rng.normal(size=10);pp,Rp,jp=c.configuration_m_to_p(p,R,joints,off);pm,Rm,jm=c.configuration_p_to_m(pp,Rp,jp,off)
 assert np.max(abs(pm-p))<1e-15 and np.max(abs(Rm-R))<1e-15 and np.max(abs(jm-joints))<1e-15
 assert np.max(abs(Xi@X-np.eye(16)))<1e-15
 assert np.max(abs(c.velocity_p_to_m(c.velocity_m_to_p(nu,X),Xi)-nu))<1e-14
 ap=c.acceleration_m_to_p(a,nu,X,Xd);assert np.max(abs(c.acceleration_p_to_m(ap,nu,Xi,Xd)-a))<1e-14
 qm=c.force_p_to_m(q,X);assert np.max(abs(c.force_m_to_p(qm,Xi)-q))<1e-14 and abs((X@nu)@q-nu@qm)<1e-12
 assert np.max(abs((X@nu)[:3]-(nu[:3]+np.cross(nu[3:6],r))))<1e-14
 Mm=c.mass_p_to_m(M,X);assert abs((X@nu)@M@(X@nu)-nu@Mm@nu)<1e-12
 hm=c.bias_p_to_m(h,M,nu,X,Xd);assert np.max(abs(X.T@(M@ap+h)-(Mm@a+hm)))<1e-12
 B=rng.normal(size=(16,6));tau=rng.normal(size=6);assert np.max(abs(c.force_p_to_m(B@tau,X)-(X.T@B)@tau))<1e-12
 assert np.max(abs(J@(X@nu)-c.jacobian_p_to_m(J,X)@nu))<1e-12
 jdv=rng.normal(size=7);assert np.max(abs(c.jdotv_p_to_m(jdv,J,nu,Xd)-(jdv+J@Xd@nu)))<1e-12
 alpha=rng.normal(size=12);Nm,cm=c.reduction_p_to_m(N,cp,nu,Xi,Xd);assert np.max(abs(Xi@(N@alpha+cp-Xd@nu)-(Nm@alpha+cm)))<1e-12
 Om,bm=c.observable_p_to_m(O,bp,nu,X,Xd);assert np.max(abs(O@ap+bp-(Om@a+bm)))<1e-12
 print("DG46RC-COMP utility: PASS")
if __name__=="__main__":main()
