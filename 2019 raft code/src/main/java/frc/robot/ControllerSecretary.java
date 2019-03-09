/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Controls controller ports
 */
public class ControllerSecretary {

	public Controller driver;
	public Controller operator1;
	public Controller operator2;
	
	public ControllerSecretary () {
		
		driver = new Controller (0);
		operator1 = new Controller (1);
		operator2 = new Controller (2);
		
	}
	
	

}
