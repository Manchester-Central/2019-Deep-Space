/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Camera;
import frc.FunctionsThatShouldBeInTheJDK;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.TalonSRX_Encoder;
import frc.robot.Controller.DPadDirection;

/**
 * Calculates new speed
 */
public class DriveBase {

    //public CANSparkMax leftFront;
    //public CANSparkMax rightFront;

    private PIDLinked pids;

    Victor leftBackVictor;
	Victor leftMidVictor;
	Victor leftFrontVictor;
	Victor rightBackVictor;
	Victor rightMidVictor;
	Victor rightFrontVictor;
	
	ChaosBetterTalonSRX rightTalonSRX;
	ChaosBetterTalonSRX leftTalonSRX;

	PIDController leftPidController;
	PIDController rightPidController;

	TalonSRX_Encoder leftEncoder;
	TalonSRX_Encoder rightEncoder;
	
	
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
	public static final double LENGTH_BETWEEN_WHEELS = 25.6;

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

       // leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        //rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
       
		
		
        

        // above is real code, below is raft, comment/uncomment to make work
		
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

		leftEncoder = new TalonSRX_Encoder(leftTalonSRX);
		rightEncoder = new TalonSRX_Encoder(rightTalonSRX);
		

		leftPidController = new PIDController(P, I, D, F, leftEncoder, leftTalonSRX);
		rightPidController = new PIDController(P, I, D, F, rightEncoder, rightTalonSRX);

		setTolerance();

		
		pids = new PIDLinked(leftPidController, rightPidController);
		
		pids.setsrxs(leftTalonSRX, rightTalonSRX);
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
		rightTalonSRX.resetEncoder();
		leftTalonSRX.resetEncoder();
		
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
		leftTalonSRX.set(leftSpeed);
		rightTalonSRX.set(rightSpeed);
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
	@Deprecated
    public void cameraDrive() {

        double[] speedValues = Camera.getDriveDirections(leftTalonSRX.get(), rightTalonSRX.get());
        setSpeed(speedValues[0] * .1D, speedValues[1] * .1D);


		
	}

	@Deprecated
	public void testMotors (ControllerSecretary cs) {
		if (cs.driver.getDPad()  == Controller.DPadDirection.UP) {
			leftTalonSRX.set(1);
		} else if (cs.driver.getDPad()  == Controller.DPadDirection.LEFT) {
			leftBackVictor.set(1);
		} else if (cs.driver.getDPad()  == Controller.DPadDirection.DOWN) {
			leftMidVictor.set(1);
		} else if (cs.driver.getDPad()  == Controller.DPadDirection.RIGHT) {
			leftFrontVictor.set(1);
		} else if (cs.driver.buttonHeld(Controller.UP_Y)) {
			rightTalonSRX.set(1);
		} else if (cs.driver.buttonHeld(Controller.LEFT_X)) {
			rightBackVictor.set(1);
		} else if (cs.driver.buttonHeld(Controller.DOWN_A)) {
			rightMidVictor.set(1);
		} else if (cs.driver.buttonHeld(Controller.RIGHT_B)) {
			rightFrontVictor.set(1);
		} else {
		  setSpeed(0.0, 0.0);
		}
	}

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

	public void startStraightCameraDriveWithPID () {

		turnAngle =  Math.toRadians(Camera.getEntry("tx").getDouble(0));
		driveDistance = 100000;

	} 

	public void straightCameraDriveWithPID () {

		double angle = Camera.getEntry("tx").getDouble(0);
		if (leftTalonSRX.getCurrentPositionInches() < driveDistance
		|| rightTalonSRX.getCurrentPositionInches() < driveDistance ){

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
					 System.out.println(driveDistance);
				}
				pids.set(driveDistance, driveDistance);
				pids.drive();
			}

		} else {

			if (turnAngle > 0) {
				if (rightTalonSRX.getCurrentPositionInches() < driveDistance
				 + (Math.tan(Math.abs(turnAngle)) * LENGTH_BETWEEN_WHEELS / 2))
					setSpeed(-FIND_ANGLE_SPEED, FIND_ANGLE_SPEED);
				else 
					setSpeed(0, 0);
			} else {
				if (leftTalonSRX.getCurrentPositionInches() < driveDistance 
				+ (Math.tan(Math.abs(turnAngle)) * LENGTH_BETWEEN_WHEELS / 2))
					setSpeed(FIND_ANGLE_SPEED, -FIND_ANGLE_SPEED);
				else 
					setSpeed(0, 0);
			}

		}

		

	}

	public void cameraDriveWithPID () {

		if (turningRight) {
			if (leftTalonSRX.getCurrentPositionInches() > arcLength)
				pids.enableSpecificPID(1);
				//pids.drive();
		} else {
			if (rightTalonSRX.getCurrentPositionInches() > arcLength)
				pids.enableSpecificPID(0);
				//pids.drive();
		}

		System.out.println (rightTalonSRX.getCurrentPositionInches() > arcLength);
		
		followTalon();

	}
	
	public void drivePID() {

		pids.drive();
		followTalon();
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
		pids.srxs[0].setAdjustment(0);
		pids.srxs[1].setAdjustment(0);
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
		pids.set(leftTalonSRX.inchesToTicks(setPoint), leftTalonSRX.inchesToTicks(setPoint));
		//leftPidController.setSetpoint(leftTalonSRX.inchesToTicks(setPoint));
		//rightPidController.setSetpoint(rightTalonSRX.inchesToTicks(setPoint));
	}


	public double getP () {return leftPidController.getP();}
    public double getI () {return leftPidController.getI();}
	public double getD () {return leftPidController.getD();}
	public double getF () {return leftPidController.getF();}
	public double getSetPoint () {return leftPidController.getSetpoint();}
	public double getError () {return leftPidController.getError();}
	public double getDistanceInchesL() { return leftTalonSRX.getCurrentPositionInches();}
	public double getDistanceTicksL() { return leftTalonSRX.getCurrentPositionTicks();}
	public double getDistanceInchesR() { return rightTalonSRX.getCurrentPositionInches();}
	public double getDistanceTicksR() { return rightTalonSRX.getCurrentPositionTicks();}
	public PIDLinked getPids () {return pids;}
	public double getArcLength () {return arcLength;}
	
//myNemChef - Chris
}
