============================
Scheduler
============================

Launch Params
=======

use_robot:
	true = Begin goto SMACH, Begin find person, Begin Navigation

use_tablet:
	true = Startup the webpages

use_error:
	true = Begin error detection

use_camera:
	true = Launch Jetson code

save_task:
	arg passed into error detection

is_test:
	arg passed into error detection
	ALWAYS SET TO FALSE

use_location:
	arg passed into error detection
	false = use only estimotes
	true  = use smart home sensors + estimotes

new_map:
	true  = Begin SLAM to create new map
	false = Use previously create maps

gui:
	arg passed into navigation

lab:
	arg passed into Jetson code
	true  = use slam map created for test lab
	false = use slam map created for smart home




