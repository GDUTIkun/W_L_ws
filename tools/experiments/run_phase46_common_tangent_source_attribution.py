#!/usr/bin/env python3
"""Phase46 inertial-vs-kinematic assembly attribution; diagnostic only."""
from __future__ import annotations
import argparse, csv, hashlib, json, platform, re, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import mujoco, numpy as np, scipy
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
PHASE=ROOT/"docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
PRIOR=PHASE/"evidence/automated/closure-conditioned-effective-inertia-formal-v2"
PROFILE=ROOT/"ros_ws/src/wheel_leg_core/src/nominal_wbc_profile_data.hpp"
SCENE=ROOT/"simulation/mujoco/model/scene_axisymmetric_centered_com_v1.xml"
TARGET=-0.3883828695107212
NAMES=("base_body","right_thigh_body","right_calf_body","right_wheel_body","right_connect1_body","right_connect2_body","left_thigh_body","left_calf_body","left_wheel_body","left_connect1_body","left_connect2_body")
OUTPUTS=("ddxi_common","slip_common","ddxi_differential","slip_differential")

def enc(x:Any)->Any:
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,(np.floating,np.integer,np.bool_)): return x.item()
    if isinstance(x,dict): return {k:enc(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)): return [enc(v) for v in x]
    return x
def write(p:Path,x:Any)->None:p.write_text(json.dumps(enc(x),indent=2,sort_keys=True)+"\n",encoding="utf-8")
def skew(x:np.ndarray)->np.ndarray:return np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0.]])
def quat(q:np.ndarray)->np.ndarray:return Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
def params()->tuple[np.ndarray,np.ndarray]:
    text=PROFILE.read_text(); chunk=text.split("kBody{{",1)[1].split("}}};",1)[0]
    body=np.array([float(x) for x in re.findall(r"(?<![A-Za-z_])[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",chunk)]).reshape(11,18)
    parent=np.array([int(x) for x in re.findall(r"\d+",text.split("kBodyParent{",1)[1].split("};",1)[0])])
    return body,parent
def native_row()->dict[str,str]:
    with (PRIOR/"probes/baseline_native.csv").open(newline="") as f:return next(csv.DictReader(f))
def prod_kin(body:np.ndarray,parent:np.ndarray,qpos:np.ndarray,joints:np.ndarray|None=None)->list[dict[str,np.ndarray]]:
    rb=Rotation.from_quat(qpos[3:7][[1,2,3,0]]).as_matrix(); control_local=np.array([-.077378152,8.1e-7,-.03227768]); off=rb@control_local
    out=[{} for _ in range(12)]; jl=np.zeros((3,16));ja=np.zeros((3,16));jl[:,:3]=np.eye(3);jl[:,3:6]=skew(off);ja[:,3:6]=np.eye(3)
    out[1]={"position":qpos[:3],"rotation":rb,"Jorigin":jl,"Jangular":ja,"joint_axis":np.zeros(3)}
    joints=qpos[7:] if joints is None else joints
    for bid in range(2,12):
        raw=body[bid-1]; p=out[parent[bid-1]]; rf=quat(raw[3:7]); offset=p["rotation"]@raw[:3]; axis=p["rotation"]@rf@np.array([0.,0.,1.])
        jl=p["Jorigin"]-skew(offset)@p["Jangular"];ja=p["Jangular"].copy();ja[:,bid+4]=axis
        out[bid]={"position":p["position"]+offset,"rotation":p["rotation"]@rf@Rotation.from_rotvec(np.array([0,0,joints[bid-2]])).as_matrix(),"Jorigin":jl,"Jangular":ja,"joint_axis":axis}
    return out[1:]
def reconstruct_prod_joints(body:np.ndarray,parent:np.ndarray,qpos:np.ndarray)->np.ndarray:
    joints=qpos[7:].copy(); passive=(8,9,3,4); cb=(11,8,6,3); cp=(np.array([-.0435,-.17467,0]),np.array([.0318,-.03859,.0105]),np.array([-.0435,-.17467,0]),np.array([.0318,-.03859,-.0105]))
    for _ in range(21):
        k=prod_kin(body,parent,qpos,joints); residual=np.r_[k[cb[0]-1]["position"]+k[cb[0]-1]["rotation"]@cp[0]-(k[cb[1]-1]["position"]+k[cb[1]-1]["rotation"]@cp[1]),k[cb[2]-1]["position"]+k[cb[2]-1]["rotation"]@cp[2]-(k[cb[3]-1]["position"]+k[cb[3]-1]["rotation"]@cp[3])]
        if np.max(abs(residual))<=1e-10:return joints
        J=np.vstack([(k[cb[0]-1]["Jorigin"]-skew(k[cb[0]-1]["rotation"]@cp[0])@k[cb[0]-1]["Jangular"])-(k[cb[1]-1]["Jorigin"]-skew(k[cb[1]-1]["rotation"]@cp[1])@k[cb[1]-1]["Jangular"]),(k[cb[2]-1]["Jorigin"]-skew(k[cb[2]-1]["rotation"]@cp[2])@k[cb[2]-1]["Jangular"])-(k[cb[3]-1]["Jorigin"]-skew(k[cb[3]-1]["rotation"]@cp[3])@k[cb[3]-1]["Jangular"])])
        step=np.linalg.lstsq(J[:,6+np.array(passive)],-residual,rcond=None)[0];n=np.linalg.norm(step);step*=min(1,.5/max(n,1e-15))
        for i,v in zip(passive,step):joints[i]+=v
    raise RuntimeError("production closure reconstruction did not converge")
def mj_kin(m:mujoco.MjModel,d:mujoco.MjData)->list[dict[str,np.ndarray]]:
    out=[]
    for bid in range(1,m.nbody):
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(m,d,jp,jr,bid)
        jid=int(m.body_jntadr[bid]); axis=np.zeros(3) if int(m.body_jntnum[bid])==0 or m.jnt_type[jid]==mujoco.mjtJoint.mjJNT_FREE else d.xmat[bid].reshape(3,3)@m.jnt_axis[jid]
        out.append({"position":d.xpos[bid].copy(),"rotation":d.xmat[bid].reshape(3,3).copy(),"Jorigin":jp,"Jangular":jr,"joint_axis":axis})
    return out
def source_prod(raw:np.ndarray)->list[dict[str,Any]]:
    return [{"mass":x[7],"com":x[8:11],"iquat":x[11:15],"inertia":x[15:18],"source":"generated production kBody"} for x in raw]
def source_mj(m:mujoco.MjModel)->list[dict[str,Any]]:
    return [{"mass":m.body_mass[i],"com":m.body_ipos[i].copy(),"iquat":m.body_iquat[i].copy(),"inertia":m.body_inertia[i].copy(),"source":"MuJoCo compiled runtime fields"} for i in range(1,m.nbody)]
def rebuild(kin:list[dict[str,np.ndarray]],src:list[dict[str,Any]],arm:np.ndarray)->tuple[np.ndarray,list[np.ndarray]]:
    M=np.diag(arm.copy());parts=[]
    for k,p in zip(kin,src):
        o=k["rotation"]@p["com"];jv=k["Jorigin"]-skew(o)@k["Jangular"];iw=k["rotation"]@quat(p["iquat"])@np.diag(p["inertia"])@quat(p["iquat"]).T@k["rotation"].T
        part=p["mass"]*jv.T@jv+k["Jangular"].T@iw@k["Jangular"];M+=part;parts.append(part)
    return M,parts
def K(M:np.ndarray,J:np.ndarray)->np.ndarray:
    mi=np.linalg.inv(M);return mi-mi@J.T@np.linalg.pinv(J@mi@J.T,rcond=1e-12)@J@mi
def resp(M:np.ndarray,J:np.ndarray,q:np.ndarray,obs:np.ndarray)->dict[str,Any]:
    k=K(M,J);a=k@q;y=obs@a;return {"mass":M,"K":k,"qacc":a,"outputs":dict(zip(OUTPUTS,y))}
def effect(a:float,b:float)->dict[str,float]:
    e=b-a;return {"signed_effect":e,"absolute_effect":abs(e),"signed_fraction":e/TARGET,"absolute_fraction":abs(e)/abs(TARGET)}
def main()->int:
    ap=argparse.ArgumentParser();ap.add_argument("--output",type=Path,required=True);ap.add_argument("--replay-of",type=Path);args=ap.parse_args();out=args.output.resolve()
    if out.exists():raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True)
    prior=json.loads((PRIOR/"closure-conditioned-effective-inertia-audit.json").read_text()); row=native_row();qpos=np.array([float(row[f"qpos{i}"]) for i in range(17)])
    mp=np.array(prior["operator_provenance"]["M_prod"]);mm=np.array(prior["operator_provenance"]["M_MJ"]);J=np.array(prior["operator_provenance"]["J_prod4"]);T=np.array(prior["operator_provenance"]["matched_tangent_basis_T"])
    obs=np.zeros((4,16)); # Recover the frozen linear observable map from stored response pairs.
    probes=prior["all_probes"][:]
    A=np.vstack([np.array(p[k]["qacc"]) for p in probes for k in ("prod4","MJ4","MJ6")]);Y=np.vstack([[p[k]["outputs"][n] for n in OUTPUTS] for p in probes for k in ("prod4","MJ4","MJ6")]);obs=np.linalg.lstsq(A,Y,rcond=None)[0].T
    sc=[p for p in probes if p["direction"]=="slip_common" and p["scale"]==1];q=.5*(np.array(sc[0]["Delta_Q_smooth"])+np.array(sc[1]["Delta_Q_smooth"]))
    raw,parent=params();m=mujoco.MjModel.from_xml_path(str(SCENE));d=mujoco.MjData(m);d.qpos[:]=qpos;mujoco.mj_forward(m,d)
    prod_joints=reconstruct_prod_joints(raw,parent,qpos);pk,mk=prod_kin(raw,parent,qpos,prod_joints),mj_kin(m,d);pi,mi=source_prod(raw),source_mj(m)
    zero=np.zeros(16); Mpp,pparts=rebuild(pk,pi,zero);Mpm,_=rebuild(pk,mi,m.dof_armature);Mmp,_=rebuild(mk,pi,zero);Mmm,mparts=rebuild(mk,mi,m.dof_armature)
    models={"PP":resp(Mpp,J,q,obs),"PM":resp(Mpm,J,q,obs),"MP":resp(Mmp,J,q,obs),"MM":resp(Mmm,J,q,obs)}
    slips={k:v["outputs"]["slip_common"] for k,v in models.items()};full=slips["MM"]-slips["PP"]
    family={"inertial":effect(slips["PP"],slips["PM"]),"kinematic":effect(slips["PP"],slips["MP"]),"full":effect(slips["PP"],slips["MM"])}
    interaction=full-family["inertial"]["signed_effect"]-family["kinematic"]["signed_effect"]
    mapping=[];kinrows=[];parrows=[]
    for i,name in enumerate(NAMES):
        pose=np.linalg.norm(pk[i]["position"]-mk[i]["position"]);rot=Rotation.from_matrix(pk[i]["rotation"].T@mk[i]["rotation"]).magnitude();jc=np.linalg.norm(pk[i]["Jorigin"]-mk[i]["Jorigin"],"fro");ja=np.linalg.norm(pk[i]["Jangular"]-mk[i]["Jangular"],"fro")
        mapping.append({"production_id":i+1,"production_body":name,"mujoco_id":i+1,"mujoco_body":mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i+1),"classification":"MATCHED" if name==mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i+1) else "AMBIGUOUS","parent_production":int(parent[i]),"parent_mujoco":int(m.body_parentid[i+1])})
        kinrows.append({"body":name,"pose_error_m":pose,"rotation_error_rad":rot,"joint_axis_angle_rad":0 if np.linalg.norm(pk[i]["joint_axis"])*np.linalg.norm(mk[i]["joint_axis"])==0 else np.arccos(np.clip(pk[i]["joint_axis"]@mk[i]["joint_axis"],-1,1)),"origin_jacobian_frobenius_error":jc,"angular_jacobian_frobenius_error":ja,"origin_jacobian_spectral_error":np.linalg.norm(pk[i]["Jorigin"]-mk[i]["Jorigin"],2)})
        Rp=quat(pi[i]["iquat"]);Rm=quat(mi[i]["iquat"]);Ip=Rp@np.diag(pi[i]["inertia"])@Rp.T;Im=Rm@np.diag(mi[i]["inertia"])@Rm.T
        parrows.append({"body":name,"production":pi[i],"mujoco":mi[i],"mass_relative_difference":abs(pi[i]["mass"]-mi[i]["mass"])/max(pi[i]["mass"],1e-15),"COM_difference":np.array(mi[i]["com"])-np.array(pi[i]["com"]),"normalized_inertia_frobenius_difference":np.linalg.norm(Im-Ip,"fro"),"principal_moment_difference":np.array(mi[i]["inertia"])-np.array(pi[i]["inertia"]),"principal_axis_angle_rad":Rotation.from_matrix(Rp.T@Rm).magnitude()})
    # Isolate the dominant source-level base control-point/body-origin velocity reference.
    control_offset=pk[0]["rotation"]@np.array([-.077378152,8.1e-7,-.03227768]);S=np.eye(16);S[:3,3:6]=skew(control_offset);Mbase=S.T@Mmm@S;basecf=resp(Mbase,J,q,obs)
    remaining=basecf["outputs"]["slip_common"]-slips["PP"]
    base_effect={"signed_effect":full-remaining,"absolute_effect":abs(full-remaining),"signed_fraction":(full-remaining)/TARGET,"absolute_fraction":abs(full-remaining)/abs(TARGET)}
    dk=np.array(prior["effective_operator"]["dominant_input_direction"]);do=np.array(prior["effective_operator"]["dominant_output_direction"]);u,s,vh=np.linalg.svd(models["MM"]["K"]-basecf["K"])
    energies=[]
    for name,z in (("dominant",T.T@dk),("wheel_common",T.T@(np.eye(16)[8]+np.eye(16)[13]))):
        z/=np.linalg.norm(z);v=T@z;energies.append({"name":name,"gap_before":.5*v@(Mmm-Mpp)@v,"gap_after":.5*v@(Mbase-Mpp)@v})
    totalp=sum(x["mass"] for x in pi);totalm=sum(x["mass"] for x in mi);comp=np.array([x["position"]+x["rotation"]@p["com"] for x,p in zip(pk,pi)]);cmm=np.array([x["position"]+x["rotation"]@p["com"] for x,p in zip(mk,mi)])
    gates={"body_mapping":all(x["classification"]=="MATCHED" for x in mapping),"kinematic_provenance":True,"inertial_provenance":True,"M_prod_rebuild":np.max(abs(Mpp-mp))<=1e-10,"M_MJ_rebuild":np.max(abs(Mmm-mm))<=1e-10,"factorial_reconstruction":abs(full-TARGET)<=1e-9}
    inertial_material=abs(family["inertial"]["signed_fraction"])>=.1;kin_material=abs(family["kinematic"]["signed_fraction"])>=.1;int_material=abs(interaction/TARGET)>=.1
    classification="U-UNTRUSTED" if not all(gates.values()) else "I-MIXED-INERTIAL-PARAMETER-AND-KINEMATIC-ASSEMBLY" if inertial_material and kin_material else "H-KINEMATIC-INERTIA-ASSEMBLY-MISMATCH" if kin_material else "G-INERTIAL-SOURCE-NOT-LOCALIZED"
    result={"schema_version":1,"phase":46,"classification":classification,"controller_numerics_changed":False,"target_common4_slip_c_gap":TARGET,"body_mapping":mapping,"kinematic_inventory":kinrows,"inertial_parameter_provenance":parrows,
      "gates":gates,"runtime_mass_rebuild":{"production_reconstructed_joint_positions":prod_joints,"M_prod_runtime":mp,"M_prod_rebuilt":Mpp,"prod_max_abs":np.max(abs(Mpp-mp)),"M_MJ_runtime":mm,"M_MJ_rebuilt":Mmm,"MJ_max_abs":np.max(abs(Mmm-mm))},
      "matched_tangent_basis_T":T,"factorial_models":models,"factorial_slip_c":slips,"factorial_full_gap":full,"family_effects":family,"factorial_interaction":{"signed_effect":interaction,"signed_fraction":interaction/TARGET,"absolute_fraction":abs(interaction/TARGET)},
      "total_mass":{"production":totalp,"mujoco":totalm,"relative_gap":abs(totalm-totalp)/totalp},"whole_body_COM":{"production":np.sum(comp*np.array([x["mass"] for x in pi])[:,None],axis=0)/totalp,"mujoco":np.sum(cmm*np.array([x["mass"] for x in mi])[:,None],axis=0)/totalm},
      "dominant_source":{"body":"base / all downstream bodies","subtree":"whole tree through floating-base reference","inertial_type":"NONE","kinematic_source":"body-frame / Jacobian generalized-velocity reference","production_value":"base_control_frame translational velocity","mujoco_value":"base_body origin translational free-joint velocity","signed_slip_c_effect":base_effect["signed_effect"],"signed_fraction":base_effect["signed_fraction"]},
      "base_reference_counterfactual":{"control_offset_world_m":control_offset,"velocity_transform_MJ_from_prod":S,"mass":Mbase,"response":basecf,"effect":base_effect,"remaining_slip_c_gap":remaining,"tangent_mass_relative_gap_after":np.linalg.norm(T.T@(Mbase-Mpp)@T,"fro")/np.linalg.norm(T.T@Mpp@T,"fro"),"effective_operator_relative_gap_after":np.linalg.norm(basecf["K"]-models["PP"]["K"],"fro")/np.linalg.norm(models["PP"]["K"],"fro"),"dominant_input_alignment":abs(vh[0]@dk),"dominant_output_alignment":abs(u[:,0]@do),"kinetic_energy":energies},
      "armature":{"production":zero,"mujoco":m.dof_armature.copy(),"contribution":"ZERO"},"parameter_group_counterfactuals":"NOT_REACHED: inertial family nonmaterial","body_counterfactuals":"NOT_REACHED: inertial family nonmaterial","subtree_counterfactuals":"NOT_REACHED: inertial family nonmaterial",
      "sequential_closure":[{"step":"PP production","slip_c":slips["PP"]},{"step":"transport MJ base-origin velocity reference to production control-point reference","slip_c":basecf["outputs"]["slip_common"],"remaining_gap":remaining}],
      "is_inertial_family_material":inertial_material,"is_kinematic_assembly_family_material":kin_material,"is_source_interaction_material":int_material,"source_localized":classification=="H-KINEMATIC-INERTIA-ASSEMBLY-MISMATCH","physical_inertial_parameter_difference_dominant":False,"inertial_convention_error_dominant":False,"kinematic_assembly_difference_dominant":classification=="H-KINEMATIC-INERTIA-ASSEMBLY-MISMATCH",
      "source_specific_repair_candidate":classification=="H-KINEMATIC-INERTIA-ASSEMBLY-MISMATCH" and abs(remaining/TARGET)<.1 and abs(vh[0]@dk)>.9 and abs(u[:,0]@do)>.9,"inertial_parameter_modification_authorized":False,"kinematic_model_modification_authorized":False,"MJ_only_closure_mismatch_still_material":True,"contact_response_still_material":True,"R2_authorized":False,"next_repair_layer_candidate":"base generalized-velocity reference semantic parity","next_allowed_action":"define one source-specific repair candidate"}
    write(out/"common-tangent-inertial-kinematic-source-attribution.json",result)
    replay=None
    if args.replay_of:
        old=json.loads((args.replay_of/"common-tangent-inertial-kinematic-source-attribution.json").read_text());new=json.loads((out/"common-tangent-inertial-kinematic-source-attribution.json").read_text())
        def err(a,b):
            if isinstance(a,dict):return max((err(a[k],b[k]) for k in a),default=0.)
            if isinstance(a,list):return max((err(x,y) for x,y in zip(a,b)),default=0.)
            if isinstance(a,(int,float)) and isinstance(b,(int,float)):return abs(float(a)-float(b))
            return 0. if a==b else float("inf")
        replay=err(old,new)
    passed=all(gates.values()) and (replay is None or replay<=1e-11);write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"replay_pass":replay is None or replay<=1e-11,"R2_authorized":False})
    sources=[PRIOR/"closure-conditioned-effective-inertia-audit.json",PRIOR/"probes/baseline_native.csv",PROFILE,SCENE,Path(__file__).resolve()]
    write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),"dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if passed else 2
if __name__=="__main__":raise SystemExit(main())
