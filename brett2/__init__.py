import roslib
try:
    roslib.load_manifest("pr2_controllers_msgs"); 
    roslib.load_manifest("move_base_msgs")
except Exception:
    print "could not import ros pr2 msgs"
    
try:
    import sensor_msgs
except ImportError:    
    roslib.load_manifest("trajectory_msgs"); 
    roslib.load_manifest("sensor_msgs"); 
    roslib.load_manifest("actionlib"); 
    roslib.load_manifest("rospy"); 
    roslib.load_manifest("geometry_msgs"); 
