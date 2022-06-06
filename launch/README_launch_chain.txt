ALMAX

1 godspeed: stuff & rviz

	1.1 erc.launch: 

		1.1.1 gazebo empty world

		1.1.2 UR3_robot.urdf/UR3_joint_limited_robot.urdf: stuff + robot (limited or not)+ joint constraints
		
			1.1.2.1 UR3_assembler_urdf: robot+environment & stuff
		
				1.1.2.1.1 ur.transmission : transmission between joints
		
				1.1.2.1.2 ur.gazebo: gazebo joint ...(names+self collision)?
		
				1.1.2.1.3 intel_realsense.urdf : camera topic source

		1.1.3 spawn robot model (node) and base_link position constraint

		1.1.4 controllers


ORIGINAL

1 simulator: stuff & rviz

	1.1 ur3.launch: 

		1.1.1 gazebo empty world

		1.1.2 ur3_upload: stuff & joint limited selection

			1.1.2.1 ur3_robot_urdf:stuff & robot & world base joint+link
		
				1.1.2.1.1 ur3_urdf: robot+environment & stuff
			
					1.1.2.1.1.1 ur.transmission : transmission between joints
			
					1.1.2.1.1.2 ur.gazebo: gazebo joint ...(names+self collision)?
			
					1.1.2.1.1.3 intel_realsense.urdf : camera topic source

		1.1.3 spawn robot model: node spawn gazebo model

		1.1.4 controllers
