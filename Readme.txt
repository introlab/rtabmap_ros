
Go to rtabmap.googlecode.com for more info.

Notes:

Check memory leaks with ROS:
  In the ROS launch file, add in the node tag : <node {...} launch-prefix="valgrind --tool=memcheck --leak-check=yes" {...}/>

Eclipse issue with ROS lib:
  Launch Eclipse with a script like the one in this folder "eclipse-launch.sh". This will setup ROS paths.
