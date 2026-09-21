"""Render actual recorded simulation states at 60 fps; never invent decisions."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,subprocess,time
from pathlib import Path
import numpy as np
import mujoco
from studio import WIDTH,HEIGHT,camera,compose

ROOT=Path(__file__).resolve().parent
def export(record):
    record=Path(record);started=time.monotonic()
    track=np.load(record/'recording.npz');times=track['times'];poses=track['qpos']
    events=[json.loads(x) for x in (record/'decisions.jsonl').read_text().splitlines()]
    report=json.loads((record/'report.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/scene.xml'));data=mujoco.MjData(model)
    renderer=mujoco.Renderer(model,height=HEIGHT,width=WIDTH);cam=camera()
    enc=['-c:v','h264_nvenc','-preset','p6','-tune','hq','-rc','vbr','-cq','17','-b:v','16M']
    writers=[];log=(record/'video.log').open('w')
    for name in ('replay.mp4','scene_clean.mp4'):
        cmd=['ffmpeg','-y','-hide_banner','-loglevel','warning','-f','rawvideo','-pix_fmt','rgb24','-s',f'{WIDTH}x{HEIGHT}','-r','60','-i','-','-an',*enc,'-pix_fmt','yuv420p','-movflags','+faststart',str(record/name)]
        writers.append(subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=log,stderr=log))
    frames=int(np.ceil(times[-1]*60))+1;idx=0;ei=-1;v=np.zeros(model.nv)
    try:
        for f in range(frames):
            t=min(f/60,times[-1])
            while idx+1<len(times)-1 and times[idx+1]<t:idx+=1
            alpha=np.clip((t-times[idx])/max(1e-9,times[idx+1]-times[idx]),0,1)
            mujoco.mj_differentiatePos(model,v,1.,poses[idx],poses[idx+1]);data.qpos[:]=poses[idx]
            mujoco.mj_integratePos(model,data.qpos,v,float(alpha));mujoco.mj_forward(model,data)
            while ei+1<len(events) and events[ei+1].get('received_at_s',0)<=t:ei+=1
            event=events[ei] if ei>=0 else {};outcome=report['outcome'] if t>times[-1]-.2 else 'running'
            renderer.update_scene(data,camera=cam);rgb=renderer.render()
            writers[1].stdin.write(rgb.tobytes())
            writers[0].stdin.write(np.asarray(compose(rgb,event,t,ei+1,outcome)).tobytes())
            if f%120==0:(record/'export_progress.json').write_text(json.dumps({'status':'rendering','frame':f,'total':frames,'percent':round(100*f/frames,1)}))
    finally:
        renderer.close()
        for p in writers:p.stdin.close()
        codes=[p.wait() for p in writers];log.close()
    if any(codes):raise RuntimeError(f'Video encoders exited {codes}; see video.log')
    info={'status':'complete','width':WIDTH,'height':HEIGHT,'fps':60,'frames':frames,'duration_s':frames/60,'playback':'1x wall time; interpolated recorded physics states; actual API decisions','codec':'h264_nvenc','export_seconds':time.monotonic()-started,'files':['replay.mp4','scene_clean.mp4']}
    (record/'export_progress.json').write_text(json.dumps(info,indent=2))
    return info

if __name__=='__main__':
    import sys
    print(json.dumps(export(sys.argv[1]),indent=2))
