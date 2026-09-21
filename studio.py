"""Shared studio camera and a 1080p HUD for faithful, wall-time video replay."""
from PIL import Image, ImageDraw, ImageFont
import mujoco

WIDTH,HEIGHT=1920,1080
FG='#314d45';MUTED='#7a887a';CARD='#f7f6ee';ACCENT='#658675';TRACK='#dfe3d9'
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONTS={}
def font(size,bold=False):
    key=(size,bold)
    if key not in FONTS:FONTS[key]=ImageFont.truetype(BOLD if bold else FONT,size)
    return FONTS[key]

def camera():
    cam=mujoco.MjvCamera();cam.lookat[:]=[.32,-.04,.22]
    cam.distance=1.8;cam.azimuth=70;cam.elevation=-25
    return cam

def compose(rgb,event,elapsed,step,outcome='running'):
    im=Image.fromarray(rgb);d=ImageDraw.Draw(im)
    def txt(x,y,s,size=18,fill=FG,bold=False):d.text((x,y),str(s),fill=fill,font=font(size,bold))
    txt(68,58,'Jev, carry the block.',46,bold=True)
    lat=event.get('latency_s',0) if event else 0
    txt(70,121,f'Two-stage policy  ·  Jev 1.13  ·  text state, no images',22,MUTED)
    left=(68,215,488,915);right=(1432,215,1852,915)
    for rect in (left,right):d.rounded_rectangle(rect,radius=22,fill=CARD)
    x,y=left[0]+28,left[1]+26
    txt(x,y,'INPUT TO JEV',17,MUTED,True);y+=37
    sampled=(event or {}).get('received_at_s',0)
    txt(x,y,f'Sampled cycle #{step:03d}',19);y+=44
    txt(x,y,'offsets · cm',17,MUTED)
    for j,a in enumerate('XYZ'):txt(x+178+j*62,y,a,17,MUTED,True)
    y+=31;state=(event or {}).get('input_state',{})
    for label,key in [('cube − pinch','cube_minus_tcp'),('pad − cube','goal_minus_cube')]:
        txt(x,y,label,17,MUTED)
        for j,v in enumerate(state.get(key,[0,0,0])):txt(x+167+j*62,y,f'{100*v:+.1f}',17)
        y+=31
    d.line((x,y+7,left[2]-28,y+7),fill=TRACK,width=1);y+=24
    txt(x,y,f'Cube bottom {100*state.get("cube_bottom_height",0):.1f} cm',17);y+=29
    txt(x,y,f'Fingers {100*state.get("fingers_opening_m",.08):.1f} cm',17);y+=29
    contacts=state.get('cube_contacts',[])
    txt(x,y,'Contact: '+('both fingers' if state.get('both_fingers_contact') else 'table' if 'world' in contacts else 'none'),17);y+=42
    d.line((x,y,left[2]-28,y),fill=TRACK,width=1);y+=22
    txt(x,y,'JEV INTENT · probabilities',16,MUTED,True);y+=38
    probs=(event or {}).get('intent_response',{}).get('answers',{}).get('intent',{}).get('probabilities',{})
    for k in ('approach','grasp','lift','carry','lower','release','withdraw','finish'):
        v=probs.get(k,0);selected=(event or {}).get('intent')==k
        txt(x,y,k.capitalize(),19,bold=selected)
        d.rounded_rectangle((x+136,y+10,x+300,y+16),3,fill=TRACK)
        if v>0:d.rounded_rectangle((x+136,y+10,x+136+max(3,164*v),y+16),3,fill=ACCENT if selected else '#a5b7b2')
        txt(x+321,y,f'{v:.0%}',17);y+=30
    txt(x,left[3]-43,'+X forward · +Y left · +Z up',15,MUTED)
    x,y=right[0]+28,right[1]+26
    txt(x,y,f'JEV MOTOR OUTPUT · #{step:03d}',16,MUTED,True);y+=38
    txt(x,y,'Four motor distributions',23,bold=True);y+=36
    txt(x,y,'14 cm/s · 100 ms target filter',15,MUTED);y+=42
    answers=(event or {}).get('motor_response',{}).get('answers',{})
    for axis,title,labels in [('x','X · forward / backward',{'positive':'Forward','negative':'Backward','stay':'Stay'}),('y','Y · left / right',{'positive':'Left','negative':'Right','stay':'Stay'}),('z','Z · up / down',{'positive':'Up','negative':'Down','stay':'Stay'}),('fingers','Fingers',{'open':'Open','close':'Close','stay':'Stay'})]:
        txt(x,y,title,17,MUTED,True);y+=27
        ans=answers.get(axis,{})
        for k,label in labels.items():
            v=ans.get('probabilities',{}).get(k,0);selected=ans.get('choice')==k
            txt(x,y,label,17,bold=selected)
            d.rounded_rectangle((x+119,y+9,x+304,y+15),3,fill=TRACK)
            if v>0:d.rounded_rectangle((x+119,y+9,x+119+max(3,185*v),y+15),3,fill=ACCENT if selected else '#a5b7b2')
            txt(x+326,y,f'{v:.0%}',16);y+=26
        y+=15
    txt(x,right[3]-34,f'API cycle {lat*1000:.0f} ms',16,MUTED)
    d.rounded_rectangle((68,979,925,1033),radius=10,fill=CARD)
    txt(87,995,'MuJoCo state → text  ·  Jev 1.13  ·  1080p / 60 fps  ·  1× playback',18,MUTED)
    txt(1590,989,f'{elapsed:05.1f} s',28)
    if outcome=='success':txt(785,160,'PLACEMENT COMPLETE',20,ACCENT,True)
    return im
