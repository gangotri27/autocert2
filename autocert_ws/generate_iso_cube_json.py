import json
from autocert_experiment.strategies.iso_cube import ISOCubeStrategy

config = {
    "strategy": "iso_cube",
    "robot_name": "open_manipulator_x",
    "pose_set": "standard",  # "standard", "extended", or "full"
    "cube_size_mm": 320,
    "cube_center": [0.0126, -0.0004, 0.1463],
    "fixed_orientation": [0,0,0,1],
    "repetitions_per_pose": 1,
    "settling_time_s": 2.0,
    "measurement_timeout_s": 5.0
}

# Create strategy and generate poses
strategy = ISOCubeStrategy(config)
poses = strategy.generate_poses()

# Convert to JSON-serializable format
pose_list = []
for p in poses:
    pose_list.append({
        "position": {"x": p.position.x, "y": p.position.y, "z": p.position.z},
        "orientation": {"x": p.orientation.x, "y": p.orientation.y,
                        "z": p.orientation.z, "w": p.orientation.w}
    })

# Save the full config including mandatory fields
config["poses"] = pose_list
output_file = "/home/ubuntu/autocert_ws/configs/open_manipulator_iso.json"
with open(output_file, "w") as f:
    json.dump(config, f, indent=2)

print(f"✅ AutoCert JSON ready: {output_file}")