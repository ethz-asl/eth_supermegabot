# Reports the state of a joystick and additional custom commands.
Header header			# timestamp in the header is the time the data is received from the joystick
sensor_msgs/Joy joy		# the state of a joysticks axes and buttons
string[] modules			# names of specific modules in the joy_manager
string[] commands		# custom service commands for the different modules defined above 
