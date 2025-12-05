#!/usr/bin/env python3
"""
Debug script to understand SmolVLA action generation
This helps you see what actions the model is generating and why the robot might barely be moving
"""

import torch
import numpy as np
import cv2
from datetime import datetime
from pathlib import Path
import math

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import build_inference_frame
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower

# Create output directories with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = Path(f"debug_output_{timestamp}")
images_dir = output_dir / "images"
images_dir.mkdir(parents=True, exist_ok=True)

# Open log file
log_file = output_dir / "debug_log.txt"
log_handle = open(log_file, "w")

def log(message):
    """Write to both console and log file"""
    print(message)
    log_handle.write(message + "\n")
    log_handle.flush()

log(f"Output directory: {output_dir}")
log(f"Images will be saved to: {images_dir}")
log(f"Log file: {log_file}\n")

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model_id = "lerobot/smolvla_base"

log("Loading model...")
model = SmolVLAPolicy.from_pretrained(model_id)
model.to(device)
log("Model loaded!\n")

preprocess, postprocess = make_pre_post_processors(
    model.config,
    model_id,
    preprocessor_overrides={"device_processor": {"device": str(device)}},
)

# Robot setup
follower_port = "/dev/ttyACM0"
follower_id = "follower"

camera_config = {
    "camera1": OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30),
}

robot_cfg = SO101FollowerConfig(port=follower_port, id=follower_id, cameras=camera_config, use_degrees=True)
robot = SO101Follower(robot_cfg)
robot.connect()

# Conversion functions for degrees <-> radians
def obs_degrees_to_radians(obs):
    """Convert joint positions from degrees to radians"""
    obs_rad = obs.copy()
    for key in obs:
        if key.endswith(".pos") and key != "gripper.pos":
            obs_rad[key] = math.radians(obs[key])
    return obs_rad

def action_radians_to_degrees(action):
    """Convert actions from radians to degrees"""
    action_deg = {}
    for key, val in action.items():
        if key.endswith(".pos") and key != "gripper.pos":
            action_deg[key] = math.degrees(val)
        else:
            action_deg[key] = val
    return action_deg

task = "pickup the bunny carefully without flailing movements and dropping it"
robot_type = "so101_follower"

# Get dataset features
action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

log("\n" + "="*80)
log("ROBOT OBSERVATION AND ACTION ANALYSIS")
log("="*80)

# Run a few steps and analyze
num_steps = 100
for step in range(num_steps):
    log(f"\n--- Step {step + 1}/{num_steps} ---")

    # Get current observation
    obs = robot.get_observation()

    # Convert degrees to radians for VLA model
    obs_radians = obs_degrees_to_radians(obs)

    # Save camera images
    for cam_key in camera_config.keys():
        if cam_key in obs:
            img = obs[cam_key]

            # Convert from tensor to numpy if needed
            if isinstance(img, torch.Tensor):
                img_np = img.cpu().numpy()
            else:
                img_np = np.array(img)

            # Save image
            img_filename = images_dir / f"step_{step:03d}_{cam_key}.jpg"

            # OpenCV expects BGR, image might be RGB and in range [0,1]
            if img_np.max() <= 1.0:
                img_np = (img_np * 255).astype(np.uint8)

            # Convert RGB to BGR for OpenCV
            if len(img_np.shape) == 3 and img_np.shape[2] == 3:
                img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            else:
                img_bgr = img_np

            cv2.imwrite(str(img_filename), img_bgr)
            log(f"Saved image: {img_filename}")

    # Print current joint positions
    log("\nCurrent joint positions (degrees):")
    for key, val in obs.items():
        if key.endswith(".pos"):
            val_rad = obs_radians[key]
            log(f"  {key}: {val:.4f}° = {val_rad:.4f} rad")

    # Build inference frame with RADIANS
    obs_frame = build_inference_frame(
        observation=obs_radians, ds_features=dataset_features, device=device, task=task, robot_type=robot_type
    )

    # Log observation frame sent to model
    log("\nObservation frame sent to model:")
    for key, val in obs_frame.items():
        if isinstance(val, torch.Tensor):
            log(f"  {key}: Tensor shape={val.shape}, min={val.min().item():.3f}, max={val.max().item():.3f}")
        else:
            log(f"  {key}: {val}")

    # Preprocess
    obs_preprocessed = preprocess(obs_frame)

    # Get action from model
    action = model.select_action(obs_preprocessed)

    # Print raw action from model (before postprocessing)
    log("\nRaw action from model (before postprocess):")
    if isinstance(action, torch.Tensor):
        action_np = action.cpu().numpy()
        log(f"  Shape: {action.shape}")
        log(f"  Values: {action_np}")
        log(f"  Min: {action_np.min():.4f}, Max: {action_np.max():.4f}, Mean: {action_np.mean():.4f}")

    # Postprocess
    action_post = postprocess(action)

    # Print postprocessed action
    log("\nPostprocessed action:")
    if isinstance(action_post, dict):
        for key, val in action_post.items():
            if isinstance(val, torch.Tensor):
                val_np = val.cpu().numpy()
                log(f"  {key}: {val_np}")
    else:
        log(f"  {action_post}")

    # Convert to robot action format
    from lerobot.policies.utils import make_robot_action
    robot_action_rad = make_robot_action(action_post, dataset_features)

    # Convert action from radians to degrees for robot
    robot_action_deg = action_radians_to_degrees(robot_action_rad)

    # Print final robot action
    log("\nFinal robot action (radians -> degrees):")
    for key in sorted(robot_action_rad.keys()):
        if key.endswith(".pos"):
            rad_val = robot_action_rad[key]
            deg_val = robot_action_deg[key]
            current_deg = obs[key]
            delta_deg = deg_val - current_deg
            log(f"  {key}: {rad_val:.4f} rad = {deg_val:.4f}° (current: {current_deg:.4f}°, delta: {delta_deg:.4f}°)")

    # Send action in DEGREES
    actual_action = robot.send_action(robot_action_deg)

    # # Wait 2 seconds to let robot move
    # import time
    # log("\nWaiting 2 seconds for movement...")
    # time.sleep(2.0)

    # Check if robot actually moved
    obs_after = robot.get_observation()
    log("\nPosition check after 2 seconds (degrees):")
    for key in sorted(robot_action_deg.keys()):
        if key.endswith(".pos"):
            before = obs[key]
            after = obs_after[key]
            delta = after - before
            status = "✓ MOVED" if abs(delta) > 0.5 else "✗ STUCK"  # Use 0.5° threshold
            log(f"  {key}: {before:.4f}° → {after:.4f}° ({delta:+.4f}°) {status}")

log("\n" + "="*80)
log("ANALYSIS SUMMARY")
log("="*80)
log("\nKey things to check:")
log("1. Are the action deltas very small? (< 0.01) - This means the model is generating tiny movements")
log("2. Are actions all very similar across steps? - The model might not be adapting to observations")
log("3. Is the model trained on your specific robot/task?")
log("4. Does your camera view match what the model was trained on?")
log(f"\nImages saved to: {images_dir}")
log(f"Log file saved to: {log_file}")
log("\nTo record your own data, use:")
log("  lerobot-record --robot.type=so101_follower --dataset.repo_id=<your_name>/<dataset_name> ...")

log("\nDisconnecting robot...")
robot.disconnect()
log_handle.close()
print(f"\n{'='*80}\nAll output saved to: {output_dir}\n{'='*80}")
