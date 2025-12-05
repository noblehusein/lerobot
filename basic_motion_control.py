#!/usr/bin/env python3
"""
Keyboard-controlled motion script for SO-101 Follower robot.

This script lets you control the robot with your keyboard:

Controls:
- 1/2: Shoulder pan left/right
- 3/4: Shoulder lift up/down  
- 5/6: Elbow flex extend/contract
- 7/8: Wrist flex up/down
- 9/0: Wrist roll left/right
- [/]: Gripper open/close
- r: Return to starting position
- s: Show current positions
- q: Quit

Make sure your robot is calibrated first using lerobot-calibrate!
"""

import time
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig


def print_controls():
    print("\n" + "="*50)
    print("KEYBOARD CONTROLS:")
    print("="*50)
    print("1/2: Shoulder pan left/right")
    print("3/4: Shoulder lift up/down")  
    print("5/6: Elbow flex extend/contract")
    print("7/8: Wrist flex up/down")
    print("9/0: Wrist roll left/right")
    print("[/]: Gripper open/close")
    print("r:   Return to starting position")
    print("s:   Show current positions")
    print("q:   Quit")
    print("="*50)


def show_positions(robot):
    observation = robot.get_observation()
    print("\nCurrent joint positions:")
    for joint_name in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]:
        current_pos = observation[f"{joint_name}.pos"]
        print(f"  {joint_name}: {current_pos:.3f}")


def main():
    # Configure the robot - update the port and ID as needed
    config = SO101FollowerConfig(
        port="/dev/ttyACM0",  # Update this to match your robot's port
        id=None,  # Use None to match your existing calibration file
        use_degrees=True,  # Your robot is calibrated in degrees
        max_relative_target=None,  # Disable safety clamping for now
        disable_torque_on_disconnect=False,  # Prevent motor overload on disconnect
    )
    
    # Create and connect to the robot
    robot = SO101Follower(config)
    print("Connecting to SO-101 Follower...")
    robot.connect()
    print("Robot connected successfully!")
    
    # Movement step size in degrees  
    step_size = 10.0
    
    try:
        # Get initial positions
        observation = robot.get_observation()
        initial_positions = {
            f"{joint}.pos": observation[f"{joint}.pos"] 
            for joint in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        }
        
        print_controls()
        show_positions(robot)
        
        current_positions = initial_positions.copy()
        
        while True:
            key = input("\nEnter command (or 'q' to quit): ").strip().lower()
            
            if not key:
                continue
                
            if key == 'q':
                break
            elif key == 's':
                show_positions(robot)
                continue
            elif key == 'r':
                print("Returning to starting position...")
                current_positions = initial_positions.copy()
            else:
                # Handle movement keys
                action_made = False
                    
                if key == '1':  # Shoulder pan left
                    current_positions["shoulder_pan.pos"] -= step_size
                    print(f"Moving shoulder pan left by {step_size}")
                    action_made = True
                elif key == '2':  # Shoulder pan right
                    current_positions["shoulder_pan.pos"] += step_size
                    print(f"Moving shoulder pan right by {step_size}")
                    action_made = True
                elif key == '3':  # Shoulder lift up
                    current_positions["shoulder_lift.pos"] += step_size
                    print(f"Moving shoulder lift up by {step_size}")
                    action_made = True
                elif key == '4':  # Shoulder lift down
                    current_positions["shoulder_lift.pos"] -= step_size
                    print(f"Moving shoulder lift down by {step_size}")
                    action_made = True
                elif key == '5':  # Elbow extend
                    current_positions["elbow_flex.pos"] -= step_size
                    print(f"Extending elbow by {step_size}")
                    action_made = True
                elif key == '6':  # Elbow contract
                    current_positions["elbow_flex.pos"] += step_size
                    print(f"Contracting elbow by {step_size}")
                    action_made = True
                elif key == '7':  # Wrist flex up
                    current_positions["wrist_flex.pos"] += step_size
                    print(f"Moving wrist flex up by {step_size}")
                    action_made = True
                elif key == '8':  # Wrist flex down
                    current_positions["wrist_flex.pos"] -= step_size
                    print(f"Moving wrist flex down by {step_size}")
                    action_made = True
                elif key == '9':  # Wrist roll left
                    current_positions["wrist_roll.pos"] -= step_size
                    print(f"Rolling wrist left by {step_size}")
                    action_made = True
                elif key == '0':  # Wrist roll right
                    current_positions["wrist_roll.pos"] += step_size
                    print(f"Rolling wrist right by {step_size}")
                    action_made = True
                elif key == '[':  # Gripper open
                    current_positions["gripper.pos"] -= step_size
                    print(f"Opening gripper by {step_size}")
                    action_made = True
                elif key == ']':  # Gripper close
                    current_positions["gripper.pos"] += step_size
                    print(f"Closing gripper by {step_size}")
                    action_made = True
                else:
                    print(f"Unknown key: {key}")
                    continue
                
            # Send the action to the robot
            if action_made or key == 'r':
                try:
                    print(f"Sending action: {current_positions}")
                    result = robot.send_action(current_positions)
                    print(f"Action result: {result}")
                    time.sleep(0.1)  # Small delay for smooth movement
                except Exception as e:
                    print(f"Error sending action: {e}")
                        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received.")
        
    except Exception as e:
        print(f"Error during motion control: {e}")
        
    finally:
        # Always disconnect the robot
        print("\nDisconnecting robot...")
        robot.disconnect()
        print("Robot disconnected. Script finished.")


if __name__ == "__main__":
    main()