#!/usr/bin/env python3
import sys
import os
import glob
import sqlite3
import csv
import math

try:
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleOdometry
except ImportError:
    print("ERROR: Could not import ROS 2 modules.")
    print("Make sure you run 'source /opt/ros/humble/setup.bash' and 'source ~/major_ws/install/setup.bash' first!")
    sys.exit(1)

def extract_bag(bag_path):
    print(f"Analyzing ROS 2 Bag: {bag_path}")
    
    db_files = glob.glob(os.path.join(bag_path, "*.db3"))
    if not db_files:
        print("ERROR: No .db3 file found! Ensure you used '-s sqlite3' when recording.")
        return
        
    db_path = max(db_files, key=os.path.getsize)
    if os.path.getsize(db_path) == 0:
        print(f"ERROR: {db_path} is 0 bytes.")
        return

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Check if tables exist
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='topics'")
    if not cursor.fetchone():
        print("ERROR: 'topics' table not found.")
        return

    # Map topic IDs to names and types
    cursor.execute("SELECT id, name, type FROM topics")
    topic_map = {row[0]: {"name": row[1], "type": row[2]} for row in cursor.fetchall()}

    # Metrics storage
    lead_to_wingman_count = 0
    wingman_to_lead_count = 0
    health_msgs_count = 0
    
    start_time_ns = None
    end_time_ns = None

    # Odometry state
    lead_pos = None
    wingman_pos = None
    min_separation = float('inf')

    timeseries_data = []

    # Iterate through all messages
    print("Processing messages (this may take a few seconds)...")
    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp ASC")
    
    for row in cursor.fetchall():
        topic_id = row[0]
        timestamp = row[1]
        data = row[2]
        
        topic_info = topic_map[topic_id]
        topic_name = topic_info["name"]
        
        # Track total time
        if start_time_ns is None:
            start_time_ns = timestamp
        end_time_ns = timestamp

        # Communication counters
        if topic_name == '/agent/lead_to_wingman':
            lead_to_wingman_count += 1
        elif topic_name == '/agent/wingman_to_lead':
            wingman_to_lead_count += 1
        elif topic_name == '/agent/health':
            health_msgs_count += 1

        # Odometry parsing
        if topic_name in ['/fmu/out/vehicle_odometry', '/px4_1/fmu/out/vehicle_odometry']:
            try:
                # Deserialize binary CDR payload
                msg = deserialize_message(data, VehicleOdometry)
                pos = (msg.position[0], msg.position[1], msg.position[2])
                
                # Check for NaNs (PX4 sends NaNs before origin is set)
                if math.isnan(pos[0]):
                    continue
                    
                if topic_name == '/fmu/out/vehicle_odometry':
                    lead_pos = pos
                else:
                    wingman_pos = pos
                    
                # Compute distance if both are known
                if lead_pos and wingman_pos:
                    dist = math.sqrt((lead_pos[0]-wingman_pos[0])**2 + (lead_pos[1]-wingman_pos[1])**2 + (lead_pos[2]-wingman_pos[2])**2)
                    if dist < min_separation:
                        min_separation = dist
                        
                    time_sec = (timestamp - start_time_ns) / 1e9
                    timeseries_data.append([
                        f"{time_sec:.2f}", 
                        f"{lead_pos[0]:.2f}", f"{lead_pos[1]:.2f}", f"{lead_pos[2]:.2f}",
                        f"{wingman_pos[0]:.2f}", f"{wingman_pos[1]:.2f}", f"{wingman_pos[2]:.2f}",
                        f"{dist:.2f}"
                    ])
                    
            except Exception as e:
                # Ignore deserialization errors for individual frames
                pass

    conn.close()

    # Calculate final summary
    execution_time = (end_time_ns - start_time_ns) / 1e9 if start_time_ns else 0.0

    # Write Summary CSV
    summary_file = f"{bag_path}_metrics_summary.csv"
    with open(summary_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Metric", "Value"])
        writer.writerow(["Total Execution Time (s)", f"{execution_time:.2f}"])
        writer.writerow(["Lead to Wingman Msgs", lead_to_wingman_count])
        writer.writerow(["Wingman to Lead Msgs", wingman_to_lead_count])
        writer.writerow(["Total Envelope Msgs", lead_to_wingman_count + wingman_to_lead_count])
        writer.writerow(["Safety Monitor Ticks", health_msgs_count])
        writer.writerow(["Minimum Separation (m)", f"{min_separation:.2f}" if min_separation != float('inf') else "N/A"])
        
    print(f"\nSaved summary to: {summary_file}")

    # Write Timeseries CSV
    timeseries_file = f"{bag_path}_odometry_timeseries.csv"
    with open(timeseries_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Time (s)", "Lead_X", "Lead_Y", "Lead_Z", "Wingman_X", "Wingman_Y", "Wingman_Z", "Separation_Distance"])
        writer.writerows(timeseries_data)
        
    print(f"Saved timeseries to: {timeseries_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_all_metrics.py <path_to_rosbag_folder>")
        sys.exit(1)
    extract_bag(sys.argv[1])
