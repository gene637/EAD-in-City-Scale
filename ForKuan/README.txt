Read me file for the Eco-Top system.

I-TSC_MaximizeThroughput_cleanedUpCode.py:

Started file off of the runner.py code from the SUMO tutorials file in traci_tls folder.

lines 52 - 66
creates files to record data for each direction.

lines 68 - 327
creates route file required for SUMO.

lines 345 - 360
count function from the paper by Y. Du

lines 363 - 437
functions for optimization 

lines 446 - 641
main function of the file.
	line 503 - 638
	main while loop of the code
		lines 507 - 541
		Signal optimization section.
		
		lines 545 - 635
		EAD section	
			lines 547 - 593
			keep track of vehicles and record in files.
			
			lines 605 - 612
			get the queue for each lane.
			
			lines 620 - 635
			calls EAD function (EAD_acceleration)
			
lines 644 - end
how to run SUMO
	line 668
	step length sets the timestep length. tripinfo-output is an output file explained on SUMO website. I'm not positive if the rest of the options do anything at all.
	
I-TSC_MaximizeThroughput_randomDepartureLane.py:

Randomizes which lane the vehicles start in, I believe. I can't really remember. The rest of the code is the same as I-TSC_MaximizeThroughput_cleanedUpCode.py.
But the line numbers are different from above.


EAD_acceleration.py:

lines 67 - 277
EAD for the East to West direction.
	lines 73 - 116
	get remaining signal time depending on the signal state for the through direction.
	
	lines 117 - 161
	get remaining signal time depending on the signal state for the left turn lane.
	
	lines 179 - 218
	change lane based on queue.
	
	lines 241 - 253
	get new vehicle velocity for CAV vehicles. Calls predicted_velocity_v1 function from EAD_regressor.py
	
	lines 258
	set new velocity on the vehicle.
	
Remaining code just repeats line 67 - 277 for each direction: West to East, North to South, South to North.


EAD_regressor.py
I never changed or did anything to this code file. I guess it just calculates the vehicle velocity for EAD.
	
