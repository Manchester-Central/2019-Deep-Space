/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import frc.Camera;
import frc.FunctionsThatShouldBeInTheJDK;
import frc.ChaosSensors.CanSparkEncoder;

/**
 * Calculates new speed
 */
public class DriveBase {

	public CANSparkMax leftFront;
	public CANSparkMax rightFront;
	public CANSparkMax leftBack;
	public CANSparkMax rightBack;
	
	public CanSparkEncoder leftEncoder;
	public CanSparkEncoder rightEncoder;

	PIDController leftPidController;
	PIDController rightPidController;
	
	private double P = 0.25;
	private double I = 0.4;
	private double D = 0;
	private double F = 0.5;

	public static final int SQUARE_COUNTER = 10;
	public static final double FIND_ANGLE_SPEED = 0.07;
	public static final double SPIN_DISTANCE = 12;

	public static final double TOLERANCE = 2.5;
	public static final double ANGLE_TOLERANCE = 1;

	public static final double ENCODER_TICKS_PER_REVOLUTION = 4100D;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = 4*Math.PI;
	public static final double HATCH_SCORING_DISTANCE = 5.0;
	public static final double LENGTH_BETWEEN_WHEELS = 28.0;

	public static final double MANUAL_CAMERA_DRIVE_PORPORTIONAL = 80D;
	// used for camera stuff

	public DriveBase() {

		leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless);
		rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
		leftBack = new CANSparkMax(PortConstants.LEFT_BACK_SPARK, MotorType.kBrushless);
		rightBack = new CANSparkMax(PortConstants.RIGHT_BACK_SPARK, MotorType.kBrushless);
		
		leftFront.setSmartCurrentLimit(50);
		rightFront.setSmartCurrentLimit(50);
		leftBack.setSmartCurrentLimit(50);
		rightBack.setSmartCurrentLimit(50);

		rightFront.setInverted(true);
		rightBack.setInverted(true);

		leftEncoder = new CanSparkEncoder(leftFront.getEncoder(), WHEEL_CIRCUMFERENCE_INCHES, ENCODER_TICKS_PER_REVOLUTION);
		rightEncoder = new CanSparkEncoder(rightFront.getEncoder(), WHEEL_CIRCUMFERENCE_INCHES, ENCODER_TICKS_PER_REVOLUTION);
		
		leftPidController = new PIDController(P, I, D, F, leftEncoder, leftFront);
		rightPidController = new PIDController(P, I, D, F, rightEncoder, rightFront);

		setTolerance();
	}
	
	public void manualFollowCamera (double joystickLeft, double joystickRight) {
		followCameraBase(Camera.getDistance(), joystickLeft, joystickRight);
	}

	public void manualFollowCameraByArea (double joystickLeft, double joystickRight) {
		followCameraBase(Camera.getDistanceFromArea(), joystickLeft, joystickRight);
	}

	private void followCameraBase (double distance, double joystickLeft, double joystickRight) {

		// fov 54 horizontal

		double p = FunctionsThatShouldBeInTheJDK.clamp(distance / MANUAL_CAMERA_DRIVE_PORPORTIONAL, 0, 1);
		double turnAmount = (Camera.GetHorizontalAngle() / 27) * p;

		double average = (joystickLeft + joystickRight) / 2;

		double resultLeft = FunctionsThatShouldBeInTheJDK.clamp(average + turnAmount, -1, 1);
		double resultRight = FunctionsThatShouldBeInTheJDK.clamp(average - turnAmount, -1, 1);

		setSpeed(resultLeft , resultRight);
		
	}

	public void resetEncoders () {
		rightEncoder.reset();
		leftEncoder.reset();
		
	}
	

    /***
	 * sets speed
	 * @param leftSpeed speed of left side wheels
	 * @param rightSpeed speed of right side wheels
	 */
    public void setSpeed (double leftSpeed, double rightSpeed) {
		
		leftSpeed = FunctionsThatShouldBeInTheJDK.clamp(leftSpeed, -1, 1);
		rightSpeed = FunctionsThatShouldBeInTheJDK.clamp(rightSpeed, -1, 1);

		
		//leftTalonSRX.set(ControlMode.PercentOutput, leftSpeed);
		leftFront.set(leftSpeed);
		leftBack.set (leftSpeed);
		rightFront.set(rightSpeed);
		rightBack.set(rightSpeed);
		//followTalon();
		
		//System.out.println(leftSpeed);

	}
	
	public String getDriveSpeeds() {

		return "Left speed = " + leftFront.get() + ", Right speed = " + rightFront.get();

	}


	public boolean withinScoringDistance (double distance) {
		return distance <= HATCH_SCORING_DISTANCE;
	}

	public boolean withinScoringDistance () {
		return Camera.getDistance() <= HATCH_SCORING_DISTANCE;
	}

	
	public double getP () {return leftPidController.getP();}
    public double getI () {return leftPidController.getI();}
	public double getD () {return leftPidController.getD();}
	public double getF () {return leftPidController.getF();}
	public double getSetPoint () {return leftPidController.getSetpoint();}
	public double getError () {return leftPidController.getError();}
	public double getDistanceInchesL() { return leftEncoder.pidGet();}
	public double getDistanceInchesR() { return rightEncoder.pidGet();}



//
//
//
//
//
// Deprecated, useless, or unused functions:
//
//
//
//
//

	public void setTolerance() {
		rightPidController.setAbsoluteTolerance(TOLERANCE);
		leftPidController.setAbsoluteTolerance(TOLERANCE);	
	}

//myNemChef - Chris - Eason - Harriet
}
