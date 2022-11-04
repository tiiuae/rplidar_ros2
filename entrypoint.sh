#!/bin/bash -e

_term() {
	# FILL UP PROCESS SEARCH PATTERN HERE TO FIND PROPER PROCESS FOR SIGINT:
	pattern="rplidar_ros2/rplidar"

	pid_value="$(ps -ax | grep $pattern | grep -v grep | awk '{ print $1 }')"
	if [ "$pid_value" != "" ]; then
		pid=$pid_value
		echo "Send SIGINT to pid $pid"
	else
		pid=1
		echo "Pattern not found, send SIGINT to pid $pid"
	fi
	kill -s SIGINT $pid
}
trap _term SIGTERM

ros-with-env ros2 launch rplidar_ros2 sensors_launch.py &
child=$!

echo "Waiting for pid $child"
wait $child
RESULT=$?

if [ $RESULT -ne 0 ]; then
		echo "ERROR: RPLidar failed with code $RESULT" >&2
		exit $RESULT
else
		echo "INFO: RPLidar finished successfully, but returning 125 code for docker to restart properly." >&2
		exit 125
fi
