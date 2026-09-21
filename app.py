"""Local simulation workbench. Startup is paused; no hardware integration."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import threading
import time
import subprocess
import asyncio
import sys
from io import BytesIO
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, Response, FileResponse, StreamingResponse
from jev_policy import evaluate,motor_context
from studio import WIDTH,HEIGHT,camera

ROOT=Path(__file__).resolve().parent
SPEED_M_S=.14
TARGET_FILTER_S=.100
SERVO_INTERVAL_S=.020
MAX_SERVO_SUBSTEP_M=.020
FINISH_HOLD_S=1.0
class Lab:
    def __init__(self):
        self.lock=threading.RLock();self.busy=False;self.running=False;self.generation=0
        self.steps=0;self.last=None;self.error='';self.jpeg=b'';self.stop=threading.Event()
        self.executing=False;self.trajectory=None;self.record_dir=None;self.frame_number=0;self.latest_video=None;self.outcome='not_run'
        self.record_samples=[];self.record_started=0.;self.export_dir=None;self.exporting=False
        self.render_fps=0.;self.frame_version=0
        self.command=None;self.cartesian_command=None;self.joint_target=None;self.finger_target=255.;self.next_servo=0.;self.task_completed_at_s=None
        videos=[]
        for path in sorted((ROOT/'experiments').glob('run_*/replay.mp4')):
            progress=path.parent/'export_progress.json'
            if not progress.exists() or json.loads(progress.read_text()).get('status')=='complete':videos.append(path)
        if videos:self.latest_video=videos[-1]
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/scene.xml'))
        self.data=mujoco.MjData(self.model)
        self.site=self.model.site('tcp').id;self.cube=self.model.body('cube').id
        self.cube_geom=self.model.geom('cube_geom').id
        self.reset()
        self.thread=threading.Thread(target=self.render_loop,daemon=True);self.thread.start()
        self.physics=threading.Thread(target=self.physics_loop,daemon=True);self.physics.start()

    def reset(self):
        with self.lock:
            self.running=False;self.generation+=1
            mujoco.mj_resetData(self.model,self.data)
            self.data.qpos[:9]=[0,0,0,-1.57079,0,1.57079,-.7853,.04,.04]
            self.data.ctrl[:]=[0,0,0,-1.57079,0,1.57079,-.7853,255]
            mujoco.mj_forward(self.model,self.data)
            self.orientation=self.data.site_xmat[self.site].reshape(3,3).copy()
            self.steps=0;self.last=None;self.error=''
            self.outcome='not_run';self.trajectory=None
            self.command=None;self.cartesian_command=None;self.joint_target=self.data.ctrl[:7].copy();self.finger_target=255.;self.next_servo=0.;self.task_completed_at_s=None

    def observe(self):
        with self.lock:return self.observation()

    def observation(self):
        tcp=self.data.site_xpos[self.site].copy();cube=self.data.xpos[self.cube].copy()
        contacts=[];contact_geoms=[]
        for c in self.data.contact:
            if self.cube_geom in (c.geom1,c.geom2):
                other=c.geom2 if c.geom1==self.cube_geom else c.geom1
                body=self.model.geom_bodyid[other]
                contacts.append(self.model.body(body).name or self.model.geom(other).name or str(other))
                contact_geoms.append(self.model.geom(other).name)
        goal=np.array([.55,.21,.035])
        targets={'above_cube':np.r_[cube[:2],.14], 'at_cube':cube,
                 'above_goal':np.r_[goal[:2],.18], 'on_goal':goal}
        return {'task':'Grasp the orange wooden cube, lift it above the blue barrier, carry it over the barrier and place it on the tan goal pad. Release the cube on the pad, then withdraw the empty gripper.','task_mode':'place',
            'source':'MuJoCo ground-truth numeric geometry and contact, not image perception',
            'units':'metres','axes':{'x':'forward','y':'left','z':'up'},
            'tcp_xyz':tcp.round(4).tolist(),'cube_xyz':cube.round(4).tolist(),'goal_cube_center_xyz':goal.tolist(),
            'cube_minus_tcp':(cube-tcp).round(4).tolist(),'goal_minus_cube':(goal-cube).round(4).tolist(),
            'named_target_offsets_from_tcp':{k:(v-tcp).round(4).tolist() for k,v in targets.items()},
            'cube_bottom_height':round(float(cube[2]-.025),4),'barrier_top_height':.09,
            'recommended_carry_cube_center_height':.18,'pregrasp_tcp_height':.14,
            'fingers_opening_m':round(float(sum(self.data.qpos[7:9])),4),'cube_contacts':sorted(set(contacts)),
            'both_fingers_contact':all(any(side in x for x in contacts) for side in ('left_finger','right_finger')),
            'cube_on_goal':bool('goal' in contact_geoms and np.linalg.norm(cube[:2]-goal[:2])<.045),
            'cube_contact_geoms':contact_geoms,
            'previous_intent':self.last['intent'] if self.last else None,'sim_time_s':float(self.data.time)}

    def ik(self,target):
        # Damped least-squares Cartesian servo; Jev chooses direction, code
        # maps chosen directions to bounded targets, preserving orientation.
        work=mujoco.MjData(self.model);work.qpos[:]=self.data.qpos
        jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
        for _ in range(100):
            mujoco.mj_forward(self.model,work)
            R=work.site_xmat[self.site].reshape(3,3)
            dr=sum(np.cross(R[:,i],self.orientation[:,i]) for i in range(3))*.5
            e=np.r_[target-work.site_xpos[self.site],dr]
            if np.linalg.norm(e[:3])<.0004 and np.linalg.norm(dr)<.015:break
            mujoco.mj_jacSite(self.model,work,jp,jr,self.site)
            J=np.vstack([jp[:,:7],jr[:,:7]])
            dq=J.T@np.linalg.solve(J@J.T+.003*np.eye(6),e)
            work.qpos[:7]=np.clip(work.qpos[:7]+np.clip(dq,-.06,.06),self.model.jnt_range[:7,0],self.model.jnt_range[:7,1])
        mujoco.mj_forward(self.model,work)
        if np.linalg.norm(target-work.site_xpos[self.site])>.01:raise ValueError('Simulation IK did not reach proposed Cartesian step')
        return work.qpos[:7].copy()

    def step(self,apply=False):
        with self.lock:
            if self.busy:return
            self.busy=True;generation=self.generation;state=self.observation()
        try:
            result=evaluate(state,observe=self.observe)
            with self.lock:
                if generation!=self.generation:return
                self.last=result;self.error=''
                result['received_at_s']=time.monotonic()-self.record_started if self.record_dir else 0
                if apply:
                    fresh=self.observation();_,reference=motor_context(fresh,result['intent'])
                    reference=np.clip(reference,[.1,-.5,.023],[.78,.5,.65])
                    if self.command is None or self.command['intent']!=result['intent']:
                        self.cartesian_command=self.data.site_xpos[self.site].copy()
                    opening={'open':255.,'close':0.,'stay':float(self.data.ctrl[7])}[result['motor']['fingers']]
                    self.executing=True
                    self.finger_target=opening
                    self.command={'intent':result['intent'],'motor':result['motor'],'reference':reference,'expires':time.monotonic()+3.0}
                    result['controller_target_xyz']=reference.tolist()
                    result['controller']='Jev direction latched until next update; Cartesian servo 14 cm/s, 100 ms target filter, intent endpoint clamp'
                    result['controller_start_observation']=fresh
                result['simulation_motion_applied']=False
            if apply:
                hold_samples=[]
                # Only a finish judgment needs a brief stationary observation.
                # Other motions overlap the next pair of API requests.
                if result['intent']=='finish':
                    with self.lock:
                        self.task_completed_at_s=time.monotonic()-self.record_started
                        self.command=None
                wait_ticks=100 if result['intent']=='finish' else (0 if self.running else 100)
                for i in range(wait_ticks):
                    with self.lock:
                        if generation!=self.generation:break
                        if result['intent']=='finish' and i%10==0:
                            o=self.observation();hold_samples.append({'time':o['sim_time_s'],'cube_z':o['cube_xyz'][2],'both_fingers_contact':o['both_fingers_contact'],'cube_on_goal':o['cube_on_goal']})
                    time.sleep(.01)
                with self.lock:
                    result['simulation_motion_applied']=True;self.steps+=1
                    result['after_observation']=self.observation()
                    if result['intent']=='finish':
                        self.running=False
                        o=result['after_observation']
                        result['hold_samples']=hold_samples
                        stable=len(hold_samples)>=9 and max(x['cube_z'] for x in hold_samples)-min(x['cube_z'] for x in hold_samples)<.003
                        goal=all(x['cube_on_goal'] for x in hold_samples) and not o['both_fingers_contact'] and o['fingers_opening_m']>.065 and o['tcp_xyz'][2]>.15
                        self.outcome='success' if stable and goal else 'unstable_placement'
                    self.executing=False
            with (ROOT/'logs/decisions.jsonl').open('a') as stream:stream.write(json.dumps(result)+'\n')
            if self.record_dir:
                with (self.record_dir/'decisions.jsonl').open('a') as stream:stream.write(json.dumps(result)+'\n')
        except Exception as exc:
            with self.lock:self.error=type(exc).__name__+': '+str(exc)[:240];self.running=False;self.outcome='error'
        finally:
            with self.lock:self.busy=False;self.executing=False

    def loop(self):
        # Warm the remote connection before starting the task clock/recording.
        # No warm-up action is applied, cached, or reused in the episode.
        with self.lock:
            self.running=False;self.busy=True;generation=self.generation
        warm_started=time.monotonic()
        try:
            evaluate(self.observe())
        except Exception as exc:
            with self.lock:self.busy=False;self.error='Jev warm-up: '+type(exc).__name__+': '+str(exc)[:180]
            return
        warmup_s=time.monotonic()-warm_started
        with self.lock:
            self.busy=False
            if generation!=self.generation:return
            self.running=True
            self.record_dir=ROOT/'experiments'/time.strftime('run_%Y%m%d_%H%M%S')
            self.record_dir.mkdir(parents=True,exist_ok=True)
            self.frame_number=0;self.outcome='running';started=time.monotonic();self.record_started=started
            self.record_samples=[(0.,self.data.qpos.copy())]
        for _ in range(100):
            with self.lock:
                if not self.running:break
            self.step(apply=True)
        with self.lock:
            self.running=False
            if self.outcome=='running':self.outcome='step_limit' if self.steps>=100 else 'paused'
            record=self.record_dir;self.record_dir=None
            self.record_samples.append((time.monotonic()-started,self.data.qpos.copy()))
            np.savez_compressed(record/'recording.npz',times=np.array([x[0] for x in self.record_samples]),qpos=np.stack([x[1] for x in self.record_samples]))
            report={'outcome':self.outcome,'steps':self.steps,'wall_seconds':time.monotonic()-started,'task_completed_at_s':self.task_completed_at_s,'pre_recording_warmup_s':warmup_s,'observation':self.observation(),'error':self.error,'physics_state_samples':len(self.record_samples),'policy':'two sequential Jev calls per update; physics and continuous Cartesian servo overlap inference; no attachment constraint','controller':{'speed_m_s':SPEED_M_S,'contact_speed_m_s':.06,'target_filter_s':TARGET_FILTER_S,'servo_interval_s':SERVO_INTERVAL_S,'max_servo_substep_m':MAX_SERVO_SUBSTEP_M,'finish_hold_s':FINISH_HOLD_S},'hold_samples':self.last.get('hold_samples',[]) if self.last else [],'simulation_solver':{'cone':'elliptic','impratio':10,'tolerance':1e-10}}
            (record/'report.json').write_text(json.dumps(report,indent=2))
        self.export_dir=record;self.exporting=True
        with (record/'export.log').open('w') as log:
            p=subprocess.run([sys.executable,str(ROOT/'export_video.py'),str(record)],stdout=log,stderr=log)
        if p.returncode==0:self.latest_video=record/'replay.mp4'
        else:self.error='Video export failed; see '+str(record/'export.log')
        self.exporting=False

    def physics_loop(self):
        while not self.stop.is_set():
            tick=time.monotonic()
            with self.lock:
                if self.running or self.executing:
                    if self.command and tick>=self.next_servo and tick<self.command['expires']:
                        self.next_servo=tick+SERVO_INTERVAL_S
                        motor=self.command['motor'];sign=np.array([{'positive':1,'negative':-1,'stay':0}[motor[a]] for a in 'xyz'])
                        error=self.command['reference']-self.cartesian_command
                        # A delayed direction may reach its endpoint while an API
                        # call is in flight. Clamp there; never infer the next intent.
                        speed=SPEED_M_S
                        if self.command['intent'] in ('grasp','lower') and self.data.site_xpos[self.site,2]<.065:speed=.06
                        delta=np.where(error*sign>0,sign*np.minimum(np.abs(error),min(MAX_SERVO_SUBSTEP_M,speed*SERVO_INTERVAL_S)),0.)
                        self.cartesian_command+=delta
                        try:self.joint_target=self.ik(self.cartesian_command)
                        except ValueError as exc:
                            self.error=str(exc);self.outcome='error';self.running=False;self.command=None
                    alpha=1-np.exp(-.004/TARGET_FILTER_S)
                    self.data.ctrl[:7]+=alpha*(self.joint_target-self.data.ctrl[:7]);self.data.ctrl[7]+=alpha*(self.finger_target-self.data.ctrl[7])
                    for _ in range(2):
                        self.data.qfrc_applied[:7]=self.data.qfrc_bias[:7]
                        mujoco.mj_step(self.model,self.data)
                    if self.record_dir:self.record_samples.append((time.monotonic()-self.record_started,self.data.qpos.copy()))
            time.sleep(max(0.,.004-(time.monotonic()-tick)))

    def render_loop(self):
        try:
            renderer=mujoco.Renderer(self.model,height=HEIGHT,width=WIDTH)
            cam=camera();view=mujoco.MjData(self.model);previous=time.monotonic()
            while not self.stop.is_set():
                tick=time.monotonic()
                with self.lock:
                    view.qpos[:]=self.data.qpos;view.qvel[:]=self.data.qvel;view.ctrl[:]=self.data.ctrl
                mujoco.mj_forward(self.model,view);renderer.update_scene(view,camera=cam);rgb=renderer.render()
                buff=BytesIO();Image.fromarray(rgb).save(buff,format='JPEG',quality=94,subsampling=0)
                self.jpeg=buff.getvalue();self.frame_version+=1
                now=time.monotonic();fps=1/max(.001,now-previous);previous=now
                self.render_fps=fps if not self.render_fps else .95*self.render_fps+.05*fps
                time.sleep(max(0.,1/30-(time.monotonic()-tick)))
            renderer.close()
        except Exception as exc:self.error='Renderer: '+type(exc).__name__+': '+str(exc)[:200]

lab=Lab();app=FastAPI()
@app.get('/')
def index():return HTMLResponse((ROOT/'index.html').read_text())
@app.get('/api/state')
def state():
    progress={}
    if lab.export_dir and (lab.export_dir/'export_progress.json').exists():
        try:progress=json.loads((lab.export_dir/'export_progress.json').read_text())
        except json.JSONDecodeError:pass
    with lab.lock:return {'mode':'SIMULATION ONLY','running':lab.running,'busy':lab.busy,'steps':lab.steps,'last':lab.last,'error':lab.error,'observation':lab.observation(),'render_ready':bool(lab.jpeg),'outcome':lab.outcome,'recording':bool(lab.record_dir),'replay_available':bool(lab.latest_video),'exporting':lab.exporting,'export':progress,'render_fps':round(lab.render_fps,1),'resolution':[WIDTH,HEIGHT]}
@app.get('/replay.mp4')
def replay():
    if not lab.latest_video:raise HTTPException(404,'No finished recording yet')
    return FileResponse(lab.latest_video,media_type='video/mp4')
@app.get('/scene_clean.mp4')
def clean_replay():
    p=lab.latest_video.parent/'scene_clean.mp4' if lab.latest_video else None
    if not p or not p.exists():raise HTTPException(404,'No clean recording yet')
    return FileResponse(p,media_type='video/mp4')
@app.get('/stream.mjpg')
def stream():
    async def frames():
        previous=-1
        while not lab.stop.is_set():
            if lab.jpeg and previous!=lab.frame_version:
                previous=lab.frame_version;frame=lab.jpeg
                yield b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n'
            await asyncio.sleep(.008)
    return StreamingResponse(frames(),media_type='multipart/x-mixed-replace; boundary=frame',headers={'Cache-Control':'no-cache','X-Accel-Buffering':'no'})
@app.get('/frame.jpg')
def frame():return Response(lab.jpeg,media_type='image/jpeg',headers={'Cache-Control':'no-store'})
@app.post('/api/{command}')
def command(command:str):
    if command=='pause':
        with lab.lock:lab.running=False;lab.generation+=1;lab.trajectory=None;lab.command=None
    elif command=='reset':
        if lab.busy:raise HTTPException(409,'Pause and wait for current request before resetting')
        lab.reset()
    elif command in ('preview','step','run','demo'):
        with lab.lock:
            if lab.busy or lab.running or lab.exporting:raise HTTPException(409,'An episode or export is already running')
            if command=='demo':lab.reset()
            lab.running=command in ('run','demo')
        target=lab.loop if command in ('run','demo') else lambda:lab.step(apply=command=='step')
        threading.Thread(target=target,daemon=True).start()
    else:raise HTTPException(404)
    return {'ok':True}
