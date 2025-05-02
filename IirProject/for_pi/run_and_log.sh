#!/bin/bash

# Run Python files in the background
python3 pi_19_udp_receive.py &
python3 pi_16_udp_receive.py &

# Give them a few seconds to start (adjust as needed)
sleep 5

# Echo ROS 2 topics to separate text files
ros2 topic echo /motor_19_voltage > motor_19_voltage_output.txt &
ros2 topic echo /motor_16_voltage > motor_16_voltage_output.txt &
#ros2 topic echo /topic_name_2 > topic2_output.txt &
#ros2 topic echo /topic_name_3 > topic3_output.txt &

echo "Python scripts started and ROS 2 topics are being logged."

# Optional: wait for all background jobs to finish
# wait

