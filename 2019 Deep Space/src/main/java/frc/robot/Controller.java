/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//This is where a comment would go
//If I HAD ONE (Timmy Turner dad)
/**
 * Assigns button values and functions.
 */
public class Controller {

    public static final int LEFT_X = 1;
    public static final int DOWN_A = 2;
    public static final int RIGHT_B = 3;
    public static final int UP_Y = 4;

    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER = 7;
    public static final int RIGHT_TRIGGER = 8;

    public static enum DPadDirection {
		LEFT, RIGHT, UP, DOWN, UP_RIGHT, DOWN_RIGHT, UP_LEFT, DOWN_LEFT, NONE
	}

    // hi

    private Joystick stick;

    public Controller(int port) {

        stick = new Joystick(port);

        stick.getRawAxis(0);
    }

    public boolean buttonPressed(int buttonNum) {
        return stick.getRawButton(buttonNum);
    }

    public double getLeftX() {
        return stick.getRawAxis(0);
    }

    public double getLeftY() {
        return -stick.getRawAxis(1);
    }

    public double getRightX() {
        return stick.getRawAxis(2);
    }

    public double getRightY() {
        return -stick.getRawAxis(3);
    }
    
    public DPadDirection getDPad() {

        int pov = stick.getPOV();
        
        switch (pov) {
		case 0:
			return DPadDirection.UP;
		case 90:
			return DPadDirection.RIGHT;
		case 180:
			return DPadDirection.DOWN;
		case 270:
			return DPadDirection.LEFT;
		default:
			return DPadDirection.NONE;

		}
	}


}
