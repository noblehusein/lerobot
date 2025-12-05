#!/usr/bin/env python3
"""
Record and replay motions for SO-101 Follower robot.

This script allows you to:
1. Record a sequence of robot positions
2. Save the recording to a file
3. Replay the recorded motion

Usage:
- Run the script and choose 'r' to record or 'p' to replay
- During recording, move the robot manually and press Enter to capture positions
- During replay, the robot will move through all recorded positions
"""

import json
import time
import os
from datetime import datetime
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig


def get_robot():
    """Initialize and connect to the robot"""
    config = SO101FollowerConfig(
        port="/dev/ttyACM0",
        id=None,
        use_degrees=True,
        max_relative_target=None,
        disable_torque_on_disconnect=False,
    )
    
    robot = SO101Follower(config)
    print("Connecting to SO-101 Follower...")
    robot.connect()
    print("Robot connected successfully!")
    return robot


def record_motion(robot, filename=None):
    """Record a sequence of robot positions"""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motion_{timestamp}.json"
    
    print(f"\n{'='*60}")
    print("RECORDING MODE")
    print(f"{'='*60}")
    print("Instructions:")
    print("1. Robot torque will be DISABLED - you can move it freely by hand")
    print("2. Move the robot to desired positions")
    print("3. Press Enter to record each position")
    print("4. Type 'done' when finished recording")
    print("5. The motion will be saved to:", filename)
    print(f"{'='*60}")
    
    # Disable torque so robot can be moved by hand
    print("\n🔓 Disabling robot torque for manual movement...")
    robot.bus.disable_torque()
    print("✓ Robot is now free to move by hand!")
    
    recorded_positions = []
    position_count = 0
    
    try:
        while True:
            # Show current position
            obs = robot.get_observation()
            current_pos = {
                joint.replace('.pos', ''): obs[joint] 
                for joint in obs.keys() if joint.endswith('.pos')
            }
            
            print(f"\nPosition #{position_count + 1}:")
            for joint, pos in current_pos.items():
                print(f"  {joint}: {pos:.2f}°")
            
            # Wait for user input
            user_input = input("\nMove robot by hand, then press Enter to record (or 'done' to finish): ").strip()
            
            if user_input.lower() == 'done':
                break
            
            # Record the position
            recorded_positions.append({
                'timestamp': time.time(),
                'positions': current_pos.copy()
            })
            position_count += 1
            print(f"✓ Position #{position_count} recorded!")
    
    except KeyboardInterrupt:
        print("\n\nRecording interrupted by user.")
    
    finally:
        # Re-enable torque
        print("\n🔒 Re-enabling robot torque...")
        robot.bus.enable_torque()
        print("✓ Robot torque re-enabled!")
    
    # Save to file
    if recorded_positions:
        motion_data = {
            'metadata': {
                'recorded_at': datetime.now().isoformat(),
                'num_positions': len(recorded_positions),
                'robot_type': 'so101_follower'
            },
            'positions': recorded_positions
        }
        
        with open(filename, 'w') as f:
            json.dump(motion_data, f, indent=2)
        
        print(f"\n✓ Motion recorded with {len(recorded_positions)} positions")
        print(f"✓ Saved to: {filename}")
        return filename
    else:
        print("\n⚠ No positions were recorded")
        return None


def list_recordings():
    """List available motion recording files"""
    motion_files = [f for f in os.listdir('.') if f.startswith('motion_') and f.endswith('.json')]
    
    if not motion_files:
        print("No motion recordings found.")
        return None
    
    print("\nAvailable recordings:")
    for i, filename in enumerate(motion_files, 1):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            num_positions = data['metadata']['num_positions']
            recorded_at = data['metadata']['recorded_at']
            print(f"  {i}. {filename} - {num_positions} positions - {recorded_at}")
        except:
            print(f"  {i}. {filename} - (corrupted file)")
    
    return motion_files


def replay_motion(robot, filename=None):
    """Replay a recorded motion"""
    print(f"\n{'='*60}")
    print("REPLAY MODE")
    print(f"{'='*60}")
    
    if filename is None:
        motion_files = list_recordings()
        if not motion_files:
            return
        
        try:
            choice = int(input("\nEnter the number of the recording to replay: ")) - 1
            if 0 <= choice < len(motion_files):
                filename = motion_files[choice]
            else:
                print("Invalid choice.")
                return
        except ValueError:
            print("Invalid input.")
            return
    
    # Load the recording
    try:
        with open(filename, 'r') as f:
            motion_data = json.load(f)
        
        positions = motion_data['positions']
        num_positions = len(positions)
        
        print(f"\nLoaded: {filename}")
        print(f"Positions: {num_positions}")
        print(f"Recorded: {motion_data['metadata']['recorded_at']}")
        
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return
    
    # Ask for replay speed
    try:
        speed = float(input("\nReplay speed (1.0 = normal, 0.5 = half speed, 2.0 = double speed): ") or "1.0")
    except ValueError:
        speed = 1.0
        print("Using normal speed (1.0)")
    
    print(f"\nStarting replay in 3 seconds...")
    time.sleep(3)
    
    try:
        for i, position_data in enumerate(positions):
            target_positions = position_data['positions']
            
            print(f"Moving to position {i+1}/{num_positions}...")
            
            # Convert to the format expected by send_action
            action = {f"{joint}.pos": pos for joint, pos in target_positions.items()}
            
            # Send the action
            robot.send_action(action)
            
            # Wait before next position (adjust by speed)
            if i < len(positions) - 1:  # Don't wait after the last position
                wait_time = 2.0 / speed  # Base wait time of 2 seconds
                time.sleep(wait_time)
        
        print("✓ Replay completed!")
        
    except KeyboardInterrupt:
        print("\n\nReplay interrupted by user.")
    except Exception as e:
        print(f"Error during replay: {e}")


def main():
    print("SO-101 Follower Motion Record & Replay")
    print("=====================================")
    
    robot = None
    
    try:
        robot = get_robot()
        
        while True:
            print("\nOptions:")
            print("  r - Record a new motion")
            print("  p - Replay a recorded motion")  
            print("  l - List available recordings")
            print("  q - Quit")
            
            choice = input("\nEnter your choice: ").strip().lower()
            
            if choice == 'q':
                break
            elif choice == 'r':
                filename = record_motion(robot)
                if filename:
                    replay_now = input(f"\nReplay the recorded motion now? (y/n): ").strip().lower()
                    if replay_now == 'y':
                        replay_motion(robot, filename)
            elif choice == 'p':
                replay_motion(robot)
            elif choice == 'l':
                list_recordings()
            else:
                print("Invalid choice. Please try again.")
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        if robot:
            print("\nDisconnecting robot...")
            try:
                robot.disconnect()
                print("Robot disconnected successfully.")
            except Exception as e:
                print(f"Disconnect note: {e}")


if __name__ == "__main__":
    main()