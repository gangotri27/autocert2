"""
full_pipeline.launch.py
=======================
Launches experiment_node -> evaluation_node for whichever robot's
URDF/SRDF you give it, then shuts down cleanly once evaluation is done.

Certification is intentionally NOT run from inside this launch file —
see run_autocert_pipeline.sh for why (ros2 launch never forwards your
terminal's stdin to a spawned subprocess) and for the actual one-command
entry point you should be using.

── NEW: per-robot test-cube geometry (opt-in, default OFF) ─────────────
By default, nothing here changes: if you don't pass auto_cube:=true or
manual cube_center/cube_size_mm args, experiment_config.yaml's own
cube_center/cube_size_mm/workspace_bounds are used exactly as before —
identical behavior to every prior run.

To test a DIFFERENT robot's URDF/SRDF without hand-tuning a cube for it:

  auto_cube:=true
    Computes a test cube from the robot's estimated max reach (derived
    generically from the URDF kinematic chain, not hardcoded to any one
    robot), and a matching workspace_bounds box sized to comfortably
    contain the generated poses. Tunable via cube_reach_fraction (how far
    out from the base, as a fraction of reach) and cube_size_fraction
    (cube size as a fraction of reach). This is a generic heuristic, not
    a guarantee of reachability for every robot topology — verify the
    run actually succeeds, and override manually if it doesn't.

  cube_center:=x,y,z  cube_size_mm:=N
    Manual override — you supply the exact geometry (e.g. verified by
    hand for a specific robot). Takes priority over auto_cube if both
    are given.

Usage (normally you don't call this directly — use run_autocert_pipeline.sh):

  ros2 launch autocert_bringup full_pipeline.launch.py \
    urdf_file:=/path/to/robot.urdf.xacro \
    srdf_file:=/path/to/robot.srdf \
    run_id:=run_2000 \
    robot_mode:=simulation \
    auto_cube:=true
"""

import os
import json
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


class NoAliasDumper(yaml.Dumper):
    def ignore_aliases(self, data):
        return True

def _write_yaml(data: dict) -> str:
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='autocert_'
    )
    yaml.dump(data, tmp, Dumper=NoAliasDumper, default_flow_style=False)
    tmp.flush()
    return tmp.name

def _load_xacro(xacro_file: str) -> str:
    import xacro
    doc = xacro.process_file(xacro_file)
    return doc.toxml()


# ── NEW: generic (non-robot-specific) reach + cube/workspace-bounds helpers ──
def _estimate_reach_m(robot_urdf) -> float:
    """Same approach as autocert_certification's RobotDataExtractor.get_reach():
    walk the kinematic chain from the root link, summing joint origin
    offsets, and take the maximum distance reached. Works generically off
    any URDF — no robot-specific assumptions.
    """
    max_distance = 0.0
    base_link = robot_urdf.get_root()

    def compute_distance(link_name, origin=(0.0, 0.0, 0.0)):
        nonlocal max_distance
        d = (origin[0] ** 2 + origin[1] ** 2 + origin[2] ** 2) ** 0.5
        max_distance = max(max_distance, d)
        for joint in robot_urdf.joints:
            if joint.parent == link_name:
                if joint.origin:
                    xyz = joint.origin.xyz if joint.origin.xyz else [0.0, 0.0, 0.0]
                    new_origin = (origin[0] + xyz[0], origin[1] + xyz[1], origin[2] + xyz[2])
                    compute_distance(joint.child, new_origin)

    compute_distance(base_link)
    return round(max_distance, 4)


def _auto_cube_geometry(reach_m: float, reach_fraction: float, size_fraction: float, min_size_mm: float):
    """Heuristic: place a test cube forward-and-up from the base at a
    fraction of the robot's estimated reach, sized as a fraction of that
    reach. This is a generic starting point, not a guarantee of
    reachability for every robot topology (e.g. wall/ceiling-mounted or
    unusual kinematic layouts) — verify the run succeeds, and use the
    manual cube_center/cube_size_mm launch args to override if it doesn't.
    """
    r = reach_m * reach_fraction
    center = [round(r * 0.7, 4), 0.0, round(r * 0.5, 4)]
    size_mm = max(min_size_mm, round(reach_m * 1000.0 * size_fraction, 2))
    return center, size_mm


def _bounds_for_cube(center, size_mm, margin_factor: float = 1.5):
    """Compute a workspace_bounds box that comfortably contains every pose
    the iso_cube strategy can generate for this center/size (standard,
    extended, or full pose sets all stay within center ± half_size per
    axis), with margin so poses aren't accidentally filtered out by
    is_within_workspace().
    """
    half = (size_mm / 1000.0) / 2.0 * margin_factor
    return {
        'x': [round(center[0] - half, 4), round(center[0] + half, 4)],
        'y': [round(center[1] - half, 4), round(center[1] + half, 4)],
        'z': [round(center[2] - half, 4), round(center[2] + half, 4)],
    }


# ── NEW: pull a named, pre-verified safe joint configuration out of the
# SRDF, if one exists, instead of letting joint_state_publisher guess
# zero/mid-range defaults. Serious MoveIt SRDFs (Panda's included) ship
# <group_state> entries specifically because the all-zeros pose is often
# NOT collision-free for a given robot's geometry (Panda's real "ready"
# pose is non-zero for exactly this reason). This is purely additive:
# robots whose SRDF has no group_state for this planning_group fall back
# to joint_state_publisher's existing default behavior, unchanged.
def _find_srdf_group_state_zeros(srdf_text: str, planning_group: str) -> dict:
    import xml.etree.ElementTree as ET
    try:
        root = ET.fromstring(srdf_text)
    except ET.ParseError:
        return {}

    candidates = [
        gs for gs in root.findall('group_state')
        if gs.attrib.get('group') == planning_group
    ]
    if not candidates:
        return {}

    # Prefer common "home"/"ready"-style names if present, else take the
    # first one defined — any named state in the SRDF was set up by
    # whoever authored it to be a valid, non-colliding configuration.
    preferred_names = ('ready', 'home', 'default', 'zero', 'up')
    chosen = None
    for name in preferred_names:
        for gs in candidates:
            if gs.attrib.get('name', '').lower() == name:
                chosen = gs
                break
        if chosen:
            break
    if chosen is None:
        chosen = candidates[0]

    zeros = {}
    for joint in chosen.findall('joint'):
        jname = joint.attrib.get('name')
        jval = joint.attrib.get('value')
        if jname is not None and jval is not None:
            try:
                zeros[jname] = float(jval)
            except ValueError:
                continue
    return zeros


LAUNCH_ARGS = [
    DeclareLaunchArgument('urdf_file',           description='Path to URDF/xacro (required)'),
    DeclareLaunchArgument('srdf_file',           description='Path to SRDF (required)'),
    DeclareLaunchArgument('planning_group',       default_value='arm'),
    DeclareLaunchArgument('end_effector_link',    default_value='wrist_roll_link'),
    DeclareLaunchArgument('reference_frame',      default_value='link1'),
    DeclareLaunchArgument('planner_id',           default_value='RRTConnect'),
    DeclareLaunchArgument('planning_time',        default_value='5.0'),
    DeclareLaunchArgument('velocity_scaling',     default_value='0.3'),
    DeclareLaunchArgument('acceleration_scaling', default_value='0.3'),
    DeclareLaunchArgument('run_id',               default_value='run_001'),
    DeclareLaunchArgument('robot_mode',           default_value='simulation'),
    DeclareLaunchArgument(
        'data_dir', default_value=os.path.expanduser('~/autocert_data'),
        description='Root output dir; experiments/ is created under it'
    ),

    # ── NEW: opt-in per-robot test geometry. All default to "off" —
    # nothing changes unless you explicitly set these.
    DeclareLaunchArgument(
        'auto_cube', default_value='false',
        description='Compute test cube + workspace bounds from URDF reach (opt-in; default off)'
    ),
    DeclareLaunchArgument(
        'cube_center', default_value='',
        description='Manual override "x,y,z" in metres. Takes priority over auto_cube.'
    ),
    DeclareLaunchArgument(
        'cube_size_mm', default_value='',
        description='Manual override cube size in mm. Must be set together with cube_center.'
    ),
    DeclareLaunchArgument('cube_reach_fraction', default_value='0.5',
                           description='auto_cube: how far out from base, as a fraction of estimated reach'),
    DeclareLaunchArgument('cube_size_fraction', default_value='0.08',
                           description='auto_cube: cube size as a fraction of estimated reach'),
    DeclareLaunchArgument('cube_min_size_mm', default_value='20.0',
                           description='auto_cube: floor on cube size regardless of reach'),

    # ── NEW: manual fallback safe-home override ─────────────────────
    # Only needed when the robot's SRDF doesn't define a usable
    # <group_state> for the planning_group (the automatic lookup above
    # covers robots that DO ship one). Format: comma-separated
    # joint_name=value pairs, e.g.:
    #   "panda_joint1=0.0,panda_joint2=-0.785398,panda_joint3=0.0,
    #    panda_joint4=-2.356194,panda_joint5=0.0,panda_joint6=1.570796,
    #    panda_joint7=0.785398"
    # Empty (default) = no effect, falls through to SRDF group_state
    # lookup, then to joint_state_publisher's own defaults — unchanged
    # from before this option existed.
    DeclareLaunchArgument(
        'initial_joint_positions', default_value='',
        description='Manual fallback "joint=value,joint=value,..." safe-home override, '
                     'takes priority over SRDF group_state lookup'
    ),
]

def generate_launch_description():

    def setup(context, *args, **kwargs):

        urdf_file      = os.path.expanduser(LaunchConfiguration('urdf_file').perform(context))
        srdf_file      = os.path.expanduser(LaunchConfiguration('srdf_file').perform(context))
        planning_group = LaunchConfiguration('planning_group').perform(context)
        ee_link        = LaunchConfiguration('end_effector_link').perform(context)
        ref_frame      = LaunchConfiguration('reference_frame').perform(context)
        planner_id     = LaunchConfiguration('planner_id').perform(context)
        planning_time  = float(LaunchConfiguration('planning_time').perform(context))
        vel_scale      = float(LaunchConfiguration('velocity_scaling').perform(context))
        acc_scale      = float(LaunchConfiguration('acceleration_scaling').perform(context))
        run_id         = LaunchConfiguration('run_id').perform(context)
        robot_mode     = LaunchConfiguration('robot_mode').perform(context)
        data_dir       = os.path.expanduser(LaunchConfiguration('data_dir').perform(context))

        auto_cube            = LaunchConfiguration('auto_cube').perform(context).strip().lower() == 'true'
        cube_center_manual    = LaunchConfiguration('cube_center').perform(context).strip()
        cube_size_manual       = LaunchConfiguration('cube_size_mm').perform(context).strip()
        cube_reach_fraction    = float(LaunchConfiguration('cube_reach_fraction').perform(context))
        cube_size_fraction     = float(LaunchConfiguration('cube_size_fraction').perform(context))
        cube_min_size_mm       = float(LaunchConfiguration('cube_min_size_mm').perform(context))

        if urdf_file.endswith('.xacro'):
            robot_description = _load_xacro(urdf_file)
        else:
            with open(urdf_file) as f:
                robot_description = f.read()

        with open(srdf_file) as f:
            robot_description_semantic = f.read()

        if not robot_description:
            raise RuntimeError(f'robot_description empty — check: {urdf_file}')

        # ── NEW: look for a named group_state in the SRDF matching this
        # planning_group, to seed joint_state_publisher with a known-safe
        # configuration instead of arbitrary zero/mid-range defaults.
        # Empty dict (no match found) = joint_state_publisher behaves
        # exactly as before this change.
        srdf_safe_zeros = _find_srdf_group_state_zeros(robot_description_semantic, planning_group)

        # Manual override takes priority — needed when the SRDF has no
        # usable group_state at all (e.g. moveit_resources_panda_moveit_config's
        # minimal panda.srdf doesn't define one).
        initial_joint_positions_manual = LaunchConfiguration('initial_joint_positions').perform(context).strip()
        if initial_joint_positions_manual:
            manual_zeros = {}
            for pair in initial_joint_positions_manual.split(','):
                pair = pair.strip()
                if not pair:
                    continue
                if '=' not in pair:
                    raise RuntimeError(
                        f"initial_joint_positions entry '{pair}' is not in 'joint=value' format"
                    )
                jname, jval = pair.split('=', 1)
                manual_zeros[jname.strip()] = float(jval.strip())
            srdf_safe_zeros = manual_zeros
            print(f'[full_pipeline] using manual initial_joint_positions override: {srdf_safe_zeros}')
        elif srdf_safe_zeros:
            print(f'[full_pipeline] using SRDF group_state as joint_state_publisher zeros '
                  f'for group "{planning_group}": {srdf_safe_zeros}')

        # ── NEW: resolve cube_center_override / cube_size_mm_override /
        # workspace_bounds_override to pass to experiment_node. All three
        # stay at their "unset" sentinel (empty string / -1.0) unless the
        # user opted in via cube_center+cube_size_mm or auto_cube:=true —
        # in which case experiment_node.py's default behavior (using
        # experiment_config.yaml as-is) is completely unaffected.
        cube_center_override_json = ''
        cube_size_mm_override     = -1.0
        workspace_bounds_override_json = ''

        if cube_center_manual and cube_size_manual:
            try:
                parts = [float(v.strip()) for v in cube_center_manual.split(',')]
                if len(parts) != 3:
                    raise ValueError('cube_center must have exactly 3 comma-separated values (x,y,z)')
                size_mm = float(cube_size_manual)
                cube_center_override_json = json.dumps(parts)
                cube_size_mm_override = size_mm
                workspace_bounds_override_json = json.dumps(_bounds_for_cube(parts, size_mm))
                print(f'[full_pipeline] manual cube override: center={parts} size_mm={size_mm}')
            except Exception as e:
                raise RuntimeError(f'Invalid cube_center/cube_size_mm override: {e}')

        elif auto_cube:
            try:
                from urdf_parser_py.urdf import URDF
                robot_urdf = URDF.from_xml_string(robot_description)
                reach_m = _estimate_reach_m(robot_urdf)
                center, size_mm = _auto_cube_geometry(
                    reach_m, cube_reach_fraction, cube_size_fraction, cube_min_size_mm
                )
                cube_center_override_json = json.dumps(center)
                cube_size_mm_override = size_mm
                workspace_bounds_override_json = json.dumps(_bounds_for_cube(center, size_mm))
                print(f'[full_pipeline] auto_cube: estimated reach={reach_m:.3f}m '
                      f'-> cube_center={center} cube_size_mm={size_mm} '
                      f'workspace_bounds={json.loads(workspace_bounds_override_json)}')
            except Exception as e:
                print(f'[full_pipeline] WARNING: auto_cube failed ({e}); '
                      f'falling back to experiment_config.yaml defaults')
        # else: nothing set -> experiment_node uses experiment_config.yaml
        # exactly as before. Zero behavior change for existing runs.

        rsp_yaml = _write_yaml({
            'robot_state_publisher': {
                'ros__parameters': {
                    'robot_description': robot_description,
                    'publish_frequency': 50.0,
                    'use_sim_time': False,
                }
            }
        })

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[rsp_yaml],
            output='screen'
        )

        # ── NEW: static 'world' -> reference_frame transform ────────────
        # Many robots' MoveIt SRDFs declare a <virtual_joint> anchoring
        # their base link to a 'world' frame (panda_moveit_config, many
        # UR MoveIt configs, etc.). MoveIt's planning_scene_monitor /
        # current_state_monitor will not consider the robot state
        # "complete" until it can actually look up that world -> base
        # transform via TF — and robot_state_publisher only publishes the
        # robot's own internal link tree, never a 'world' anchor. Without
        # this, execution_node spins forever logging:
        #   "The complete state of the robot is not yet known. Missing virtual_joint"
        # and can never plan. OMX's SRDF happens not to declare such a
        # virtual_joint, which is why this went unnoticed until testing
        # against Panda/UR5 — it's a real generalization gap, not
        # robot-specific. Publishing this identity transform is a no-op
        # for robots that don't need it (an unused extra TF frame), and
        # fixes every robot whose SRDF does.
        world_to_ref_frame_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_reference_frame_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', ref_frame],
            output='screen'
        )

        jsp_params = {
            'robot_description': robot_description,
            'source_list': ['/simulated_joint_states'],
            'rate': 30,
        }
        if srdf_safe_zeros:
            jsp_params['zeros'] = srdf_safe_zeros

        jsp_yaml = _write_yaml({
            'joint_state_publisher': {
                'ros__parameters': jsp_params
            }
        })

        jsp_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[jsp_yaml],
            condition=IfCondition(PythonExpression(["'", robot_mode, "' == 'simulation'"])),
            output='screen'
        )

        # ── NEW: repeated seed publish on /simulated_joint_states ─────────
        # joint_state_publisher's 'zeros' parameter (used above, via
        # srdf_safe_zeros / initial_joint_positions) is the documented way
        # to set a non-zero starting pose, but it has a known ROS2 quirk:
        # it can silently fail to apply depending on how parameters get
        # declared, with no error — you just see all-zero /joint_states.
        # This was confirmed empirically (echoed /joint_states showed ~0.0
        # for every joint despite 'zeros' being set) on a Panda run where
        # the true all-zeros pose self-collides (panda_hand vs panda_link5
        # /panda_link7), so relying on 'zeros' alone left the robot stuck.
        #
        # execution_node already subscribes to /simulated_joint_states
        # (it's in jsp's source_list above) specifically to merge in state
        # updates. Publishing our safe pose there directly does not depend
        # on joint_state_publisher's parameter handling at all.
        #
        # IMPORTANT #1: publish repeatedly (--rate), not --once.
        # `ros2 topic pub --once` sends exactly one message and exits
        # immediately — but ROS2 pub/sub discovery/connection setup takes
        # a moment, so a --once publish frequently completes *before* the
        # subscriber has actually connected, silently dropping the message
        # (confirmed: this was tried first and /joint_states stayed at
        # ~0.0 even though the publish process ran without error).
        #
        # IMPORTANT #2: publish continuously (no --times limit), not for
        # a fixed few seconds. This was also tried and confirmed
        # insufficient: a 5-second burst (--times 25 @ 5Hz) delivered the
        # values correctly (verified via `ros2 topic echo /joint_states`),
        # but joint_state_publisher reverts source-tracked joints back to
        # its own internally-generated defaults the moment the source
        # stops publishing. Since experiment_node's first move attempt
        # doesn't happen until its own 20s TimerAction fires, a short
        # burst decays back to the colliding pose in the gap before that
        # first attempt ever occurs. Keeping this running for the whole
        # pipeline (it gets torn down along with everything else at
        # shutdown) avoids that gap entirely.
        #
        # No-op (nothing published) if no override / SRDF group_state was
        # found, so robots that never needed this are unaffected.
        seed_pose_actions = []
        if srdf_safe_zeros:
            joint_names = list(srdf_safe_zeros.keys())
            joint_positions = [srdf_safe_zeros[n] for n in joint_names]
            names_yaml = '[' + ', '.join(f"'{n}'" for n in joint_names) + ']'
            positions_yaml = '[' + ', '.join(str(v) for v in joint_positions) + ']'
            seed_pose_publish = ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub',
                    '--rate', '5',
                    '/simulated_joint_states', 'sensor_msgs/msg/JointState',
                    f"{{name: {names_yaml}, position: {positions_yaml}}}"
                ],
                output='screen',
                condition=IfCondition(PythonExpression(["'", robot_mode, "' == 'simulation'"]))
            )
            # Fire after jsp/execution_node are up, well before
            # experiment_node's own 20s TimerAction.
            seed_pose_actions.append(TimerAction(period=10.0, actions=[seed_pose_publish]))

        exec_yaml = _write_yaml({
            'execution_node': {
                'ros__parameters': {
                    'robot_description':          robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'planning_group':             planning_group,
                    'end_effector_link':          ee_link,
                    'reference_frame':            ref_frame,
                    'planner_id':                 planner_id,
                    'planning_time':              planning_time,
                    'velocity_scaling':           vel_scale,
                    'acceleration_scaling':       acc_scale,
                    'robot_mode':                 robot_mode,
                }
            }
        })

        execution_node = Node(
            package='autocert_execution',
            executable='execution_node',
            name='execution_node',
            parameters=[exec_yaml],
            output='screen'
        )

        exp_share  = get_package_share_directory('autocert_experiment')
        exp_config = os.path.join(exp_share, 'config', 'experiment_config.yaml')

        experiment_csv  = os.path.join(data_dir, 'experiments', f'{run_id}.csv')
        evaluation_json = os.path.join(data_dir, 'experiments', f'{run_id}_evaluation.json')

        # ── Stage 1: experiment_node ────────────────────────────────────
        experiment_node_action = Node(
            package='autocert_experiment',
            executable='experiment_node',
            name='experiment_node',
            arguments=[
                '--config', exp_config,
                '--output', experiment_csv,
                '--run-id', run_id,
            ],
            parameters=[{
                'planning_group':       planning_group,
                'end_effector_link':    ee_link,
                'reference_frame':      ref_frame,
                'planner_id':           planner_id,
                'planning_time':        planning_time,
                'velocity_scaling':     vel_scale,
                'acceleration_scaling': acc_scale,
                'robot_mode':           robot_mode,
                'run_id':               run_id,
                # NEW — see experiment_node.py's declare_parameter calls
                # for these. Empty string / -1.0 (the defaults set above
                # when neither auto_cube nor manual cube args are given)
                # means "no override", so existing behavior is unchanged.
                'cube_center_override':       cube_center_override_json,
                'cube_size_mm_override':      cube_size_mm_override,
                'workspace_bounds_override':  workspace_bounds_override_json,
            }],
            output='screen'
        )

        experiment_stage = TimerAction(
            period=20.0,
            actions=[experiment_node_action]
        )

        # ── Stage 2: evaluation_node — starts once experiment_node exits ──
        evaluation_node_action = Node(
            package='autocert_evaluation',
            executable='evaluation_node',
            name='evaluation_node',
            arguments=[
                '--input', experiment_csv,
                '--output', evaluation_json,
                '--run-id', run_id,
            ],
            output='screen'
        )

        run_evaluation_after_experiment = RegisterEventHandler(
            OnProcessExit(
                target_action=experiment_node_action,
                on_exit=[
                    LogInfo(msg=f'experiment_node finished — starting evaluation for {run_id}'),
                    evaluation_node_action,
                ]
            )
        )

        # ── Stage 3: shut the launch down once evaluation_node exits ──
        # robot_state_publisher / joint_state_publisher / execution_node
        # never exit on their own, so without this the process hangs
        # forever. run_autocert_pipeline.sh waits for THIS process (the
        # `ros2 launch` invocation) to exit before it runs
        # certification_node in the foreground, in the same terminal.
        shutdown_after_evaluation = RegisterEventHandler(
            OnProcessExit(
                target_action=evaluation_node_action,
                on_exit=[
                    LogInfo(msg=f'evaluation_node finished for {run_id} — '
                                f'shutting down launch, certification runs next'),
                    EmitEvent(event=Shutdown(reason=f'Experiment+evaluation complete for {run_id}')),
                ]
            )
        )

        return [
            robot_state_publisher,
            world_to_ref_frame_tf,
            jsp_node,
            execution_node,
            *seed_pose_actions,
            experiment_stage,
            run_evaluation_after_experiment,
            shutdown_after_evaluation,
        ]

    return LaunchDescription(LAUNCH_ARGS + [OpaqueFunction(function=setup)])