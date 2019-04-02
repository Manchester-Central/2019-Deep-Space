/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import frc.Camera;
import frc.FunctionsThatShouldBeInTheJDK;
import frc.ChaosSensors.CanSparkEncoder;
import frc.ChaosSensors.ChaosBetterCANSpark;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.TalonSRX_Encoder;
import frc.ChaosSensors.TalonSRX_Encoder.ParamType;

/**
 * Calculates new speed
 */
public class DriveBase {

   
    private PIDLinked pids;

	// This is raft code
    Victor leftBackVictor;
	Victor leftMidVictor;
	Victor leftFrontVictor;
	Victor rightBackVictor;
	Victor rightMidVictor;
	Victor rightFrontVictor;

	ChaosBetterTalonSRX rightTalonSRX;
	ChaosBetterTalonSRX leftTalonSRX;
	
	TalonSRX_Encoder leftEncoderRaft;
	TalonSRX_Encoder rightEncoderRaft;
	

	PIDController leftPidController;
	PIDController rightPidController;

	
	
	private double P = 0.25;
	private double I = 0.4;
	private double D = 0;
	private double F = 0.5;
	private double setPoint = 24.0;

	private double minAngleFound;
	private int i;
	private double[] squareSum;

	private double driveDistance;
	private double turnAngle;

	public static final int SQUARE_COUNTER = 10;
	public static final double FIND_ANGLE_SPEED = 0.07;
	public static final double SPIN_DISTANCE = 12;
	
	public static final double TOLERANCE = 2.5;
	public static final double ANGLE_TOLERANCE = 1;

	public static final double ENCODER_TICKS_PER_REVOLUTION = 4100D;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = 4*Math.PI;
	public static final double LENGTH_BETWEEN_WHEELS = 28.0;
	
	public static final double MANUAL_CAMERA_DRIVE_PORPORTIONAL = 35.0;
	public static final double HATCH_SCORING_DISTANCE = 5.0;
	public boolean withinHatchRange = false;

	// used for camera stuff
	private boolean turningRight;
	private double distance;
	private double arcLength;
	// used for camera stuff 

    public DriveBase() {

		minAngleFound = 90D;
		squareSum = new double[SQUARE_COUNTER];
		i = 0;

		driveDistance = 0;

		turningRight = false;
		turnAngle = 100000;
		
		
		rightTalonSRX = new ChaosBetterTalonSRX(PortConstants.RIGHT_CAN_TALON,
		 WHEEL_CIRCUMFERENCE_INCHES, ENCODER_TICKS_PER_REVOLUTION, false);
		
		leftTalonSRX = new ChaosBetterTalonSRX(PortConstants.LEFT_CAN_TALON,
		 WHEEL_CIRCUMFERENCE_INCHES, ENCODER_TICKS_PER_REVOLUTION, true);

		leftTalonSRX.set(ControlMode.PercentOutput, 0);

		leftBackVictor = new Victor(PortConstants.LEFT_BACK_TALON);
		leftMidVictor = new Victor(PortConstants.LEFT_MID_TALON);
		leftFrontVictor = new Victor(PortConstants.LEFT_FRONT_TALON);
		
		rightBackVictor = new Victor(PortConstants.RIGHT_BACK_TALON);
		rightMidVictor = new Victor(PortConstants.RIGHT_MID_TALON);
		rightFrontVictor = new Victor(PortConstants.RIGHT_FRONT_TALON);
		
		rightBackVictor.setInverted(true);
		rightMidVictor.setInverted(true);
		rightFrontVictor.setInverted(true);
		
		leftTalonSRX.setInverted(false);
		rightTalonSRX.setInverted(true);
		
		leftTalonSRX.enableCurrentLimit(true);
		rightTalonSRX.enableCurrentLimit(true);
		
		leftTalonSRX.configContinuousCurrentLimit(30, 0);
		rightTalonSRX.configContinuousCurrentLimit(30, 0);
		
		leftTalonSRX.configPeakCurrentDuration(50, 0);
		rightTalonSRX.configPeakCurrentDuration(50, 0);

		rightTalonSRX.configPeakCurrentLimit(30, 0);
		leftTalonSRX.configPeakCurrentLimit(30, 0);
		
		rightTalonSRX.configClosedloopRamp (1, 0);
		leftTalonSRX.configClosedloopRamp (1, 0);

		leftEncoderRaft = new TalonSRX_Encoder(leftTalonSRX, ParamType.distance);
		rightEncoderRaft = new TalonSRX_Encoder(rightTalonSRX, ParamType.distance);
		


		//Raft
		leftPidController = new PIDController(P, I, D, F, leftEncoderRaft, leftTalonSRX);
		rightPidController = new PIDController(P, I, D, F, rightEncoderRaft, rightTalonSRX);

		setTolerance();

		
		pids = new PIDLinked(leftPidController, rightPidController);
		
		pids.setSparks(leftTalonSRX, rightTalonSRX);

		Robot.describePID(leftPidController, "leftDrivePID", leftEncoderRaft.pidGet(), leftTalonSRX.getPIDWrite());

		Robot.describePID(rightPidController, "rightDrivePID", rightEncoderRaft.pidGet(), rightTalonSRX.getPIDWrite());
	}

	public void describeSelf () {
		Robot.describePID(leftPidController, "leftDrivePID", leftEncoderRaft.pidGet(), leftTalonSRX.getPIDWrite());
		//System.out.println ("leftAdjustment: " + leftTalonSRX.getAdjustment() + "\t");

		Robot.describePID(rightPidController, "rightDrivePID", rightEncoderRaft.pidGet(), rightTalonSRX.getPIDWrite());
		//System.out.println ("rightAdjustment: " + rightTalonSRX.getAdjustment() + "\t");

	}

	public void manualFollowCamera (double joystickLeft, double joystickRight) {
		followCameraBase(Camera.getDistance(), joystickLeft, joystickRight);
	}

	public void manualFollowCameraByArea (double joystickLeft, double joystickRight) {
		followCameraBase(Camera.getDistanceFromArea(), joystickLeft, joystickRight);
	}

	private void followCameraBase (double distance, double joystickLeft, double joystickRight) {

		// fov 54 horizontal
	
		withinHatchRange = distance <= HATCH_SCORING_DISTANCE;	
		
		double p = FunctionsThatShouldBeInTheJDK.clamp(
			distance / MANUAL_CAMERA_DRIVE_PORPORTIONAL, 0, 1);
		double turnAmount = (Camera.GetHorizontalAngle() / 29.8) * p;

		double average = (joystickLeft + joystickRight) / 2;


		double resultLeft = FunctionsThatShouldBeInTheJDK.clamp(average + turnAmount, -1, 1);
		double resultRight = FunctionsThatShouldBeInTheJDK.clamp(average - turnAmount, -1, 1);


		setSpeed(resultLeft , resultRight);
		
	}

	public void resetSquareSum () {

		for (int i = 0; i < SQUARE_COUNTER; i++) {
			squareSum[i] = 90;
		}

		turningRight = true;
		minAngleFound = 90;
	}

	private void addValueToSquareSum (double value) {
		for (int i = 0; i < SQUARE_COUNTER - 1; i++) {
			squareSum[i + 1] = i; 
		}

		squareSum[0] = value;
	}

	public double getAverageCamValue () {
		double sum = 0;
		for (double x : squareSum) {
			sum += x;
		}
		return sum / SQUARE_COUNTER;
	}

	public boolean squareWithVisionTarget () {
		addValueToSquareSum(Camera.getEntry("ty").getDouble(20));
		
		i++;
		if (i % SQUARE_COUNTER != 0) {
			return true;
		}

		if (getAverageCamValue() < minAngleFound) {
			minAngleFound = getAverageCamValue();
			if (turningRight) {
				setSpeed (FIND_ANGLE_SPEED, -FIND_ANGLE_SPEED);
			} else {
				setSpeed (-FIND_ANGLE_SPEED, FIND_ANGLE_SPEED);
			}
			return true;
		}

		if (turningRight) {
			turningRight = false;
			return true;
		}

		setSpeed(0.0, 0.0);

		return false;
	}

	public void resetEncoders () {
		rightEncoderRaft.reset();
		leftEncoderRaft.reset();
		
	}
	

    /***
	 * sets speed
	 * @param leftSpeed speed of left side wheels
	 * @param rightSpeed speed of right side wheels
	 */
    public void setSpeed (double leftSpeed, double rightSpeed) {
		
		leftSpeed = FunctionsThatShouldBeInTheJDK.clamp(leftSpeed, -1, 1);
		rightSpeed = FunctionsThatShouldBeInTheJDK.clamp(rightSpeed, -1, 1);

		
		leftTalonSRX.set(ControlMode.PercentOutput, leftSpeed);
		rightTalonSRX.set(ControlMode.PercentOutput, rightSpeed);
		// leftFront.set(leftSpeed);
		// leftBack.set (leftSpeed);
		// rightFront.set(rightSpeed);
		// rightBack.set(rightSpeed);
		followTalon();
		
		//System.out.println(leftSpeed);

	}

	public void followTalon () {

		double leftSpeed = leftTalonSRX.get();
		double rightSpeed = rightTalonSRX.get();

		leftBackVictor.set(leftSpeed);
		leftMidVictor.set(leftSpeed);
		leftFrontVictor.set(leftSpeed);

		rightBackVictor.set(rightSpeed);
		rightMidVictor.set(rightSpeed);
		rightFrontVictor.set(rightSpeed);
		
	}
	
	public String getDriveSpeeds() {

		return "Left speed = " + leftTalonSRX.get() + ", Right speed = " + rightTalonSRX.get();

	}



	/***
	 * drive based on the camera autonomously
	 */
	//@Deprecated
    public void cameraDrive() {

        double[] speedValues = Camera.getDriveDirections(leftTalonSRX.get(), rightTalonSRX.get());
        //setSpeed(speedValues[0], speedValues[1]);
		System.out.println (speedValues[0] + ", " + speedValues[1]);

		
	}

	// @Deprecated
	// public void testMotors (ControllerSecretary cs) {
	// 	if (cs.driver.getDPad()  == Controller.DPadDirection.UP) {
	// 		leftTalonSRX.set(0.4);
	// 	} else if (cs.driver.getDPad()  == Controller.DPadDirection.LEFT) {
	// 		leftBackVictor.set(0.4);
	// 	} else if (cs.driver.getDPad()  == Controller.DPadDirection.DOWN) {
	// 		leftMidVictor.set(0.4);
	// 	} else if (cs.driver.getDPad()  == Controller.DPadDirection.RIGHT) {
	// 		leftFrontVictor.set(0.4);
	// 	} else if (cs.driver.buttonHeld(Controller.UP_Y)) {
	// 		rightTalonSRX.set(0.4);
	// 	} else if (cs.driver.buttonHeld(Controller.LEFT_X)) {
	// 		rightBackVictor.set(0.4);
	// 	} else if (cs.driver.buttonHeld(Controller.DOWN_A)) {
	// 		rightMidVictor.set(0.4);
	// 	} else if (cs.driver.buttonHeld(Controller.RIGHT_B)) {
	// 		rightFrontVictor.set(0.4);
	// 	} else {
	// 		leftTalonSRX.set(0);
	// 		leftBackVictor.set(0);
	// 		leftMidVictor.set(0);
	// 		leftFrontVictor.set(0);
	// 		rightTalonSRX.set(0);
	// 		rightBackVictor.set(0);
	// 		rightMidVictor.set(0);
	// 		rightFrontVictor.set(0);
	// 	}
	// }

	public void initializeCameraDrive () {

		
		
		double horizontalAngle = Camera.getEntry("tx").getDouble(0D);
		double sign = FunctionsThatShouldBeInTheJDK.getSign(horizontalAngle);
		double absoluteHorizontalAngle = sign * horizontalAngle;
		double absCamAngleRadians = Math.toRadians (absoluteHorizontalAngle);

		turningRight = sign > 0D;

		arcLength = ((Math.PI/2) - absCamAngleRadians) * LENGTH_BETWEEN_WHEELS;
		distance = Camera.getDistance() - (LENGTH_BETWEEN_WHEELS* Math.sin(absCamAngleRadians));

		double driveDistance = distance + arcLength;

		pids.set(driveDistance, driveDistance);

		if (turningRight)
			pids.enableSpecificPID(0);
		else
			pids.enableSpecificPID(1);
	}

	public void resetCameraDrivePID () {

		resetEncoders();
		turnAngle =  Math.toRadians(Camera.getEntry("tx").getDouble(0));
		driveDistance = 100000;

	} 

	public void straightCameraDriveWithPID () {

		double angle = Camera.getEntry("tx").getDouble(0);
		if (leftEncoderRaft.pidGet() < driveDistance
		|| rightEncoderRaft.pidGet() < driveDistance ){

			if (Math.abs(angle) > ANGLE_TOLERANCE && !leftPidController.isEnabled() 
			&& !rightPidController.isEnabled()) {
				if (angle > 0) 
					setSpeed(FIND_ANGLE_SPEED, -FIND_ANGLE_SPEED);
				else
					setSpeed(-FIND_ANGLE_SPEED, FIND_ANGLE_SPEED);
			} else {
				if (driveDistance == 100000) {
					setSpeed(0, 0);
					resetEncoders();
					
					//y/x = z/x, tan(theta) x = z
					driveDistance = Camera.getDistance()
					 - (Math.tan(Math.abs(turnAngle)) * LENGTH_BETWEEN_WHEELS / 2);
					 //System.out.println(driveDistance);
				}
				pids.set(driveDistance, driveDistance);
				pids.drive();
			}

		} else {

			if (turnAngle > 0) {
				if (rightEncoderRaft.pidGet() < driveDistance
				 + (Math.tan(Math.abs(turnAngle)) * LENGTH_BETWEEN_WHEELS / 2))
					setSpeed(-FIND_ANGLE_SPEED, FIND_ANGLE_SPEED);
				else 
					setSpeed(0, 0);
			} else {
				if (leftEncoderRaft.pidGet() < driveDistance 
				+ (Math.tan(Math.abs(turnAngle)) * LENGTH_BETWEEN_WHEELS / 2))
					setSpeed(FIND_ANGLE_SPEED, -FIND_ANGLE_SPEED);
				else 
					setSpeed(0, 0);
			}

		}

		

	}

	public void cameraDriveWithPID () {

		if (turningRight) {
			if (leftEncoderRaft.pidGet() > arcLength)
				pids.enableSpecificPID(1);
				//pids.drive();
		} else {
			if (rightEncoderRaft.pidGet() > arcLength)
				pids.enableSpecificPID(0);
				//pids.drive();
		}

		//System.out.println (rightEncoder.pidGet() > arcLength);
		
		//followTalon();

	}
	
	public void drivePID() {

		pids.drive();
		//followTalon();
		//leftPidController.enable();
		//System.out.println (leftTalonSRX.getCurrentPositionInches());
		//rightPidController.enable();
		//setSpeed(leftTalonSRX.getPIDWrite(), rightTalonSRX.getPIDWrite());
		//setSpeed(leftTalonSRX.getPIDWrite(), 0d/*rightTalonSRX.getPIDWrite()*/);

		//leftTalonSRX.set(leftPidController.get());
		
		
		// System.out.println(leftPID.getPIDSpeed(leftTalonSRX.getCurrentPositionInches()));
		// setSpeed(leftPID.getPIDSpeed(leftTalonSRX.getCurrentPositionInches()), 0);
	
	}

	public void stopDrivePID() {
		//leftPidController.disable();
		//rightPidController.disable();
		pids.sparks[0].setAdjustment(0);
		pids.sparks[1].setAdjustment(0);
		pids.stop();
	}

	public void setTolerance() {
		rightPidController.setAbsoluteTolerance(TOLERANCE);
		leftPidController.setAbsoluteTolerance(TOLERANCE);
		
	}

	public void setPIDValues(double p, double i, double d, double f) {
		pids.setPIDValues(p, i, d, f);
	}

	public void setDriveDistance(double setPoint) {
		pids.set(setPoint, setPoint);
		//leftPidController.setSetpoint(leftTalonSRX.inchesToTicks(setPoint));
		//rightPidController.setSetpoint(rightTalonSRX.inchesToTicks(setPoint));
	}

	public boolean withinHatchRange()  { return withinHatchRange; }
	public double getP () {return leftPidController.getP();}
    public double getI () {return leftPidController.getI();}
	public double getD () {return leftPidController.getD();}
	public double getF () {return leftPidController.getF();}
	public double getSetPoint () {return leftPidController.getSetpoint();}
	public double getError () {return leftPidController.getError();}
	public double getDistanceInchesL() { return leftEncoderRaft.pidGet();}
	//public double getDistanceTicksL() { return leftTalonSRX.getCurrentPositionTicks();}
	public double getDistanceInchesR() { return rightEncoderRaft.pidGet();}
	//public double getDistanceTicksR() { return rightTalonSRX.getCurrentPositionTicks();}
	public PIDLinked getPids () {return pids;}
	public double getArcLength () {return arcLength;}
	
//myNemChef - Chris - Eason
}
