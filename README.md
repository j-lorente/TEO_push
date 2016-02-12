# TEO_push
Push-recovery experiments with humanoid robot TEO (UC3M)

Instructions:

	1. Terminal 1:	ssh 2.2.2.52 / ssh locomotion

	2. Terminal 1:	sudo chmod 777 /dev/ttyUSB0

	3. Terminal 1:	yarpdev --subdevice xsensmtx --device inertial --name /inertial --verbose

	4. Terminal 2:	ssh 2.2.2.52 / ssh locomotion

	5. Terminal 2:	launchLocomotion

	6. Launch exe



Note:

Lines marked with "This is for plot with python" are for using along with the external python application in order to plot the PID results in a graph.
