echo "Launching: roscore"
xterm -title "roscore" -e bash -c "roscore; bash" &
echo "Waiting 2s left"
sleep 2
echo "Launching: Turtlesim node"
xterm -title "turtlesim node" -e bash -c "rosrun turtlesim turtlesim_node; bash" &

echo "Waiting 2s left"
sleep 2
echo "Launching: robot node"
xterm -title "robot node" -e bash -c "rosrun challenge robot; bash" &

echo "Waiting 2s left"
sleep 2
echo "Launching: give_dirs node"
xterm -title "give_dirs" -e bash -c "rosrun challenge give_dirs; bash" &

echo "Waiting 2s left"
sleep 2
echo "Launching: services node"
xterm -title "services node" -e bash -c "rosrun challenge services_node.py; bash" &

echo "Waiting 2s left"
sleep 2
echo "Launching: rosbridge"
xterm -title "rosbridge" -e bash -c "roslaunch rosbridge_server rosbridge_websocket.launch port:=9091"

