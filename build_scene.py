"""Prepare a Franka Panda tabletop scene using official Menagerie assets."""
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent
source=ROOT/'assets/franka_emika_panda'
tree=ET.parse(source/'panda.xml');r=tree.getroot()
# panda.xml is written under assets/, so keep mesh paths relative to this repo.
r.find('compiler').set('meshdir','franka_emika_panda/assets')
r.remove(r.find('keyframe'))
hand=r.find(".//body[@name='hand']")
ET.SubElement(hand,'site',name='tcp',pos='0 0 0.103',size='.008',rgba='.1 .8 .8 .7')
# The site remains available to the controller without a visible marker.
hand.find("site[@name='tcp']").set('rgba','0 0 0 0')
for parent in r.iter():
    for light in list(parent.findall('light')):parent.remove(light)
tree.write(ROOT/'assets/panda.xml')
(ROOT/'assets/scene.xml').write_text('''<mujoco model="Junqi Jev MuJoCo Lab">
<include file="panda.xml"/>
<option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10" tolerance="1e-10"/>
<visual>
 <global offwidth="1920" offheight="1080"/>
 <quality shadowsize="4096" offsamples="4"/>
 <headlight ambient="0.55 0.55 0.55" diffuse="0.15 0.15 0.15" specular="0.06 0.06 0.06"/>
 <map znear="0.01" zfar="15" fogstart="8" fogend="12"/>
 <rgba fog="0.81 0.82 0.79 1"/>
</visual>
<asset>
 <texture name="studio" type="skybox" builtin="gradient" rgb1="0.75 0.77 0.73" rgb2="0.88 0.88 0.85" width="512" height="3072"/>
 <material name="desk_matte" rgba="0.78 0.79 0.75 1" specular="0.12" shininess="0.2"/>
</asset>
<worldbody>
 <light pos="-.5 -1 2.5" dir=".2 .3 -1" diffuse=".35 .35 .33" specular=".08 .08 .08" directional="true" castshadow="true"/>
 <light pos="1 1 2" dir="-.3 -.3 -1" diffuse=".1 .1 .12" castshadow="false"/>
 <geom name="studio_floor" type="plane" pos="0 0 -.075" size="0 0 .1" rgba=".72 .74 .7 1"/>
 <geom name="table" type="box" pos=".35 0 -.03" size=".64 .55 .03" material="desk_matte" friction="1 .01 .001"/>
 <geom name="goal" type="box" pos=".55 .21 .005" size=".07 .07 .005" rgba=".78 .58 .43 1"/>
 <geom name="barrier" type="box" pos=".55 .03 .045" size=".13 .012 .045" rgba=".31 .48 .53 1"/>
 <body name="cube" pos=".48 -.20 .025"><freejoint name="cube_free"/>
 <geom name="cube_geom" type="box" size=".025 .025 .025" mass=".08" rgba=".78 .36 .12 1" friction="1.4 .01 .001"/>
 </body>
</worldbody></mujoco>''')
print('scene prepared')
