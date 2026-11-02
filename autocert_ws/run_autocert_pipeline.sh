#!/usr/bin/env bash
#
# run_autocert_pipeline.sh
# =========================
# The one command that runs the whole AutoCert pipeline AND supports
# real interactive input for certificate metadata.
#
# Why this script exists instead of doing everything inside
# full_pipeline.launch.py: `ros2 launch` never connects a spawned
# subprocess's stdin to your real terminal (this is a documented
# limitation of the ros2/launch project, not something fixable with a
# flag like emulate_tty — that flag only affects stdout formatting).
# So certification_node's input() calls can never receive real
# keystrokes while running as a Node inside `ros2 launch`.
#
# The fix: run the ROS graph (experiment + evaluation) under
# `ros2 launch`, let it shut itself down when evaluation finishes, and
# THEN run certification_node directly as a plain child process of
# this script. At that point it is a completely normal foreground
# process with a real, fully-connected terminal, exactly like running
# `ros2 run autocert_certification certification_node ...` by hand —
# because that is exactly what this script does.
#
# ── NEW: per-robot test-cube geometry (opt-in, default OFF) ────────────
# Pass auto_cube:=true to compute a test cube + workspace bounds from
# whatever URDF/SRDF you give (generic reach-based heuristic), or
# cube_center:=x,y,z cube_size_mm:=N to set it manually. Leaving both
# unset (the default) means every existing behavior is identical to
# before this change — experiment_config.yaml's own OMX-tuned cube is
# used exactly as it always was.
#
# Usage: same key:=value style as the launch file, e.g.
#
#   ./run_autocert_pipeline.sh \
#     urdf_file:=/home/ubuntu/omx_ws/src/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf.xacro \
#     srdf_file:=/home/ubuntu/omx_ws/src/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x/open_manipulator_x.srdf \
#     planning_group:=arm end_effector_link:=end_effector_link reference_frame:=link1 \
#     planner_id:=RRTConnect planning_time:=5.0 run_id:=run_2005 robot_mode:=simulation
#
#   # Testing a different robot, letting the pipeline size its own test cube:
#   ./run_autocert_pipeline.sh \
#     urdf_file:=/path/to/ur5.urdf.xacro srdf_file:=/path/to/ur5.srdf \
#     planning_group:=manipulator end_effector_link:=tool0 reference_frame:=base_link \
#     run_id:=ur5_run_001 robot_mode:=simulation auto_cube:=true
#
# Must be run in a real, foreground terminal session (not backgrounded
# with `&`, not under nohup, not in a CI runner) for the certification
# step's interactive prompts to work. In those non-interactive contexts
# certification_node still runs fine — it just falls back to defaults
# for any missing metadata field instead of hanging.

set -euo pipefail

# ── Defaults (mirrors full_pipeline.launch.py's DeclareLaunchArgument defaults) ──
urdf_file=""
srdf_file=""
planning_group="arm"
end_effector_link="wrist_roll_link"
reference_frame="link1"
planner_id="RRTConnect"
planning_time="5.0"
velocity_scaling="0.3"
acceleration_scaling="0.3"
run_id="run_001"
robot_mode="simulation"
data_dir="$HOME/autocert_data"
robot_metadata_file=""
limits_file=""
robot_image=""
robot_image_2=""

# NEW — default off, identical behavior to before unless explicitly set
auto_cube="false"
cube_center=""
cube_size_mm=""
cube_reach_fraction="0.5"
cube_size_fraction="0.08"
cube_min_size_mm="20.0"

# NEW — manual safe-home override, empty by default (no effect)
initial_joint_positions=""

# ── Parse key:=value args (same style as `ros2 launch`) ─────────────
for arg in "$@"; do
    if [[ "$arg" != *":="* ]]; then
        echo "ERROR: unrecognized argument '$arg' (expected key:=value)" >&2
        exit 1
    fi
    key="${arg%%:=*}"
    val="${arg#*:=}"
    case "$key" in
        urdf_file)             urdf_file="$val" ;;
        srdf_file)              srdf_file="$val" ;;
        planning_group)         planning_group="$val" ;;
        end_effector_link)      end_effector_link="$val" ;;
        reference_frame)        reference_frame="$val" ;;
        planner_id)              planner_id="$val" ;;
        planning_time)           planning_time="$val" ;;
        velocity_scaling)        velocity_scaling="$val" ;;
        acceleration_scaling)    acceleration_scaling="$val" ;;
        run_id)                  run_id="$val" ;;
        robot_mode)              robot_mode="$val" ;;
        data_dir)                data_dir="$val" ;;
        robot_metadata_file)     robot_metadata_file="$val" ;;
        limits_file)             limits_file="$val" ;;
        robot_image)             robot_image="$val" ;;
        robot_image_2)           robot_image_2="$val" ;;
        auto_cube)               auto_cube="$val" ;;
        cube_center)             cube_center="$val" ;;
        cube_size_mm)            cube_size_mm="$val" ;;
        cube_reach_fraction)     cube_reach_fraction="$val" ;;
        cube_size_fraction)      cube_size_fraction="$val" ;;
        cube_min_size_mm)        cube_min_size_mm="$val" ;;
        initial_joint_positions) initial_joint_positions="$val" ;;
        *)
            echo "ERROR: unknown argument key '$key'" >&2
            exit 1
            ;;
    esac
done

if [[ -z "$urdf_file" || -z "$srdf_file" ]]; then
    echo "ERROR: urdf_file:=... and srdf_file:=... are required" >&2
    exit 1
fi

data_dir="${data_dir/#\~/$HOME}"

# ── NEW: same fix, applied to every other path-like arg. Bash only
# auto-expands a leading '~' in true assignment syntax (VAR=~/path); it
# does NOT expand it in 'key:=~/path' tokens (the ':=' breaks the
# special case), so anything typed as 'urdf_file:=~/...' arrives here
# with a literal, unexpanded '~'. data_dir already got this treatment;
# extending it to the rest closes the same gap for every other path.
urdf_file="${urdf_file/#\~/$HOME}"
srdf_file="${srdf_file/#\~/$HOME}"
robot_metadata_file="${robot_metadata_file/#\~/$HOME}"
limits_file="${limits_file/#\~/$HOME}"
robot_image="${robot_image/#\~/$HOME}"
robot_image_2="${robot_image_2/#\~/$HOME}"

# Resolve package share dir the same way get_package_share_directory() does.
pkg_share() {
    python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g('$1'))"
}

cert_share="$(pkg_share autocert_certification)"
if [[ -z "$robot_metadata_file" ]]; then
    robot_metadata_file="$cert_share/config/robot_metadata.yaml"
fi
if [[ -z "$limits_file" ]]; then
    limits_file="$cert_share/config/limits.yaml"
fi

experiments_dir="$data_dir/experiments"
certificates_dir="$data_dir/certificates"
mkdir -p "$experiments_dir" "$certificates_dir"

evaluation_json="$experiments_dir/${run_id}_evaluation.json"
certificate_json="$certificates_dir/${run_id}.json"
certificate_pdf="$certificates_dir/${run_id}.pdf"

echo "============================================================"
echo " AutoCert pipeline — run_id: $run_id"
echo " Stage 1/2 (ros2 launch): experiment_node -> evaluation_node"
if [[ "$auto_cube" == "true" ]]; then
    echo " auto_cube: ON — test cube will be computed from URDF reach"
elif [[ -n "$cube_center" && -n "$cube_size_mm" ]]; then
    echo " cube override: center=($cube_center) size_mm=$cube_size_mm"
fi
echo "============================================================"

# ── Stage 1+2: experiment + evaluation, under ros2 launch ──────────
# This call blocks until the launch file's internal Shutdown event
# fires (i.e. once evaluation_node has finished), or until you Ctrl-C.
ros2launch_args=(
    urdf_file:="$urdf_file"
    srdf_file:="$srdf_file"
    planning_group:="$planning_group"
    end_effector_link:="$end_effector_link"
    reference_frame:="$reference_frame"
    planner_id:="$planner_id"
    planning_time:="$planning_time"
    velocity_scaling:="$velocity_scaling"
    acceleration_scaling:="$acceleration_scaling"
    run_id:="$run_id"
    robot_mode:="$robot_mode"
    data_dir:="$data_dir"
    auto_cube:="$auto_cube"
    cube_reach_fraction:="$cube_reach_fraction"
    cube_size_fraction:="$cube_size_fraction"
    cube_min_size_mm:="$cube_min_size_mm"
)
# NOTE: cube_center / cube_size_mm are only appended when actually set.
# ros2 launch's own CLI parser rejects an empty value after ':=' (e.g.
# 'cube_center:=' with nothing after it) as a malformed argument, even
# though the Python launch file declares a default_value='' for it —
# the crash happens at ros2 launch's tokenizing step, before Python ever
# sees it. Passing these unconditionally (even as "") is what broke
# every run, cube-related or not.
if [[ -n "$cube_center" ]]; then
    ros2launch_args+=(cube_center:="$cube_center")
fi
if [[ -n "$cube_size_mm" ]]; then
    ros2launch_args+=(cube_size_mm:="$cube_size_mm")
fi
if [[ -n "$initial_joint_positions" ]]; then
    ros2launch_args+=(initial_joint_positions:="$initial_joint_positions")
fi

ros2 launch autocert_bringup full_pipeline.launch.py "${ros2launch_args[@]}"

launch_status=$?
if [[ $launch_status -ne 0 ]]; then
    echo "ERROR: ros2 launch exited with status $launch_status — aborting before certification." >&2
    exit $launch_status
fi

if [[ ! -f "$evaluation_json" ]]; then
    echo "ERROR: expected evaluation output not found: $evaluation_json" >&2
    echo "       experiment_node or evaluation_node likely failed — check the log above." >&2
    exit 1
fi

echo
echo "============================================================"
echo " Stage 2/2 (plain foreground process): certification_node"
echo " This step can prompt you interactively for any metadata"
echo " field missing from: $robot_metadata_file"
echo "============================================================"

# ── Stage 3: certification, run directly — real terminal, real stdin ──
cert_cmd=(
    ros2 run autocert_certification certification_node
    --input "$evaluation_json"
    --output "$certificate_json"
    --pdf "$certificate_pdf"
    --robot-metadata "$robot_metadata_file"
    --limits "$limits_file"
    --run-id "$run_id"
    --urdf "$urdf_file"
    --srdf "$srdf_file"
    --robot-mode "$robot_mode"
)
if [[ -n "$robot_image" ]]; then
    cert_cmd+=(--robot-image "$robot_image")
fi
if [[ -n "$robot_image_2" ]]; then
    cert_cmd+=(--robot-image-2 "$robot_image_2")
fi

"${cert_cmd[@]}"
cert_status=$?

echo
if [[ $cert_status -eq 0 && -f "$certificate_pdf" ]]; then
    echo "============================================================"
    echo " DONE — certificate PDF: $certificate_pdf"
    echo "============================================================"
else
    echo "ERROR: certification_node exited with status $cert_status, or PDF was not written." >&2
    echo "       Expected PDF at: $certificate_pdf" >&2
    exit 1
fi