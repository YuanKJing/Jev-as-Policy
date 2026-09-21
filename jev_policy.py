"""Two sequential Jev calls, text-only state. No robot/hardware imports."""
import ast
import os
import time
import msgspec
import numpy as np
from pathlib import Path
from typesafe_sdk import TypeSafeClient, Choice

MODEL = 'jev-1.13.0'
KEY_FILE = Path(os.environ.get('JEV_KEY_FILE', Path.home() / '.config/typesafe/jev-key.py'))
INTENTS = {
    'approach': 'Cube not held, not on goal; gripper not horizontally aligned over cube. Move open gripper above cube.',
    'grasp': 'Cube not held, not on goal; gripper horizontally aligned over cube. Descend to surround cube, then close fingers.',
    'lift': 'Cube held by BOTH fingers, below carrying height, and NOT above goal. Raise vertically before transporting sideways. If already above goal choose lower instead.',
    'carry': 'Cube held and raised to carrying height, but not horizontally aligned over goal. Transport above goal.',
    'lower': 'Cube held, horizontally aligned over goal, but not resting on goal. Lower it onto pad.',
    'release': 'Cube resting on goal, fingers still closed or gripping it. Open fingers in place.',
    'withdraw': 'Cube resting on goal, fingers open, gripper still near cube. Raise empty gripper.',
    'finish': 'Cube resting on goal, fingers open, empty gripper already withdrawn above it.'}
CLIENT = None


def load_key():
    for name in ('TYPESAFE_API_KEY', 'JEV_API_KEY'):
        if os.environ.get(name):
            return os.environ[name]
    if not KEY_FILE.exists():
        raise ValueError(f'No Jev API key found. Set TYPESAFE_API_KEY or JEV_API_KEY, or create {KEY_FILE}')
    text=KEY_FILE.read_text().strip()
    values=[]
    try:
        tree=ast.parse(text)
        for node in tree.body:
            if isinstance(node,ast.Assign):
                value=ast.literal_eval(node.value)
                if isinstance(value,str): values.append(value)
            elif isinstance(node,ast.Expr) and isinstance(node.value,ast.Constant) and isinstance(node.value.value,str):
                values.append(node.value.value)
    except (SyntaxError,ValueError):
        pass
    key=values[0] if len(values)==1 else text
    if any(c.isspace() for c in key) or len(key)<20:
        raise ValueError('API key file format unsupported; value was not logged')
    return key


def geometry(state):
    """Known geometry arithmetic belongs in the harness, not in the model."""
    tcp=np.array(state['tcp_xyz']);cube=np.array(state['cube_xyz']);goal=np.array(state['goal_cube_center_xyz'])
    held=bool(state['both_fingers_contact']);on_goal=bool(state['cube_on_goal'])
    return {'cube_held_by_both_fingers':held,'cube_resting_on_goal':on_goal,
        'gripper_horizontally_aligned_with_cube':bool(np.max(np.abs(tcp[:2]-cube[:2]))<.012),
        'cube_horizontally_aligned_with_goal':bool(np.max(np.abs(cube[:2]-goal[:2]))<.014),
        'cube_at_carrying_height':bool(cube[2]>=.165),
        'fingers_open':bool(state['fingers_opening_m']>.065),
        'gripper_at_grasp_height':bool(abs(tcp[2]-cube[2])<.009),
        'gripper_withdrawn':bool(tcp[2]>.15),'cube_contacts':state['cube_contacts']}


def motor_context(state,intent):
    tcp=np.array(state['tcp_xyz']);cube=np.array(state['cube_xyz']);goal=np.array(state['goal_cube_center_xyz'])
    targets={'approach':np.r_[cube[:2],max(.14,float(cube[2]+.10))],
        'grasp':cube.copy(),'lift':np.r_[tcp[:2],tcp[2]+max(0.,.18-cube[2])],
        'carry':tcp+np.r_[goal[:2]-cube[:2],.18-cube[2]],
        'lower':tcp+goal-cube,'release':tcp.copy(),
        'withdraw':np.r_[tcp[:2],.20],'finish':tcp.copy()}
    target=targets[intent]
    if state.get('task_mode')=='lift' and intent=='finish':target=tcp.copy()
    # Approach above obstacles before any lateral translation from a low pose.
    if intent=='approach' and tcp[2]<.12:target=np.r_[tcp[:2],.14]
    offset=target-tcp
    positive=('forward','left','up');negative=('backward','right','down')
    relation={a:('aligned' if abs(offset[i])<.007 else (positive[i] if offset[i]>0 else negative[i])) for i,a in enumerate('xyz')}
    return {'selected_intent':intent,'intent_definition':INTENTS[intent],
        'target_relative_to_gripper':relation,'target_offset_m':dict(zip('xyz',offset.round(4).tolist())),
        'facts':geometry(state)},target


def evaluate(state,observe=None):
    global CLIENT
    started=time.perf_counter()
    if CLIENT is None:CLIENT=TypeSafeClient(api_key=load_key(),timeout=20.)
    lift_only=state.get('task_mode')=='lift'
    intents={k:INTENTS[k] for k in ('approach','grasp','lift')} if lift_only else dict(INTENTS)
    if lift_only:
        intents['lift']='Cube held by BOTH fingers, below carrying height. Raise vertically.'
        intents['finish']='Cube held by BOTH fingers and already raised to carrying height. Hold the cube still with fingers closed; lifting task complete.'
    intent_state={'task':state['task'],'observed_facts':geometry(state),
        'previous_intent':state.get('previous_intent'),'object_height_m':state['cube_xyz'][2]}
    intent=CLIENT.system_one(model=MODEL,state=intent_state,questions={
        'intent':Choice(instructions='Select the next manipulation intent whose conditions match the observed facts. Use current contact and alignment, not an imagined successful action. '+('Task is ONLY lift: finish only when cube held by both fingers and raised to carrying height.' if lift_only else 'Once cube is held and aligned over goal, lower it; do not choose lift again while lowering. Once cube rests on goal: release, withdraw, finish.'),criteria=intents)})
    intent_done=time.perf_counter();chosen=intent.choices['intent'].choice
    # The arm keeps moving during intent inference. Sample actual geometry again
    # before the dependent motor request instead of using an already old frame.
    motor_observation=observe() if observe else state
    motor_state,target=motor_context(motor_observation,chosen)
    motor_state['intent_definition']=intents[chosen]
    motor_state['task']=state['task']
    questions={}
    for axis,pos,neg in zip('xyz',('forward','left','up'),('backward','right','down')):
        questions[axis]=Choice(instructions=f'Which direction along the {axis.upper()} axis moves the gripper toward the current target? Read ONLY target_relative_to_gripper.{axis}. If aligned, stay. Do not choose a different target or intent.',criteria={
            'positive':f'The target is {pos}; move {pos} along +{axis}.',
            'negative':f'The target is {neg}; move {neg} along -{axis}.',
            'stay':f'The target is aligned along {axis}; no movement along this axis.'})
    questions['fingers']=Choice(instructions='Choose fingers for the selected intent: approach=open; grasp=open while descending and close only when at grasp height; lift/carry/lower=close; release/withdraw=open. '+('finish=close (hold lifted cube).' if lift_only else 'finish=open.')+' Use facts.gripper_at_grasp_height for grasp.',criteria={'open':'Open or keep fingers open.','close':'Close or keep fingers closed.','stay':'Keep current finger opening.'})
    motors=CLIENT.system_one(model=MODEL,state=motor_state,questions=questions)
    return {'model':MODEL,'intent':chosen,'motor':{k:v.choice for k,v in motors.choices.items()},
            'intent_response':msgspec.to_builtins(intent),'motor_response':msgspec.to_builtins(motors),
            'latency_s':time.perf_counter()-started,'intent_latency_s':intent_done-started,
            'motor_latency_s':time.perf_counter()-intent_done,'input_state':state,
            'intent_request_state':intent_state,'motor_request_state':motor_state,
            'motor_observation':motor_observation,
            'reference_target_xyz':target.tolist(),'api_calls':2,'policy_version':'streaming_text_geometry_v3'}
