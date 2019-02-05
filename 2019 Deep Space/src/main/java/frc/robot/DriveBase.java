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

/**
 * Calculates new speed
 */
public class DriveBase {

    //public CANSparkMax leftFront;
    //public CANSparkMax rightFront;

    private KYSPID leftPID;
    private KYSPID rightPID;
    private PIDLinked pid;

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
	
	public static final double TOLERANCE = 0.1;

	public static final double ENCODER_TICKS_PER_REVOLUTION = 4100D;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = 4*Math.PI;
	public static final double LENGTH_BETWEEN_WHEELS = 25.6;

	// used for camera stuff
	private boolean turningLeft;
	private double distance;
	private double arcLength;
	// used for camera stuff

    public DriveBase() {

		turningLeft = false;

       // leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        //rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
       
        leftPID = new KYSPID(P, I, D, F,setPoint);
		//pid = new PIDLinked(leftPID, rightPID);
		
		
        

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
		
		//System.out.println(leftSpeed);

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

	public void cameraDriveWithPID () {
		
		double horizontalAngle = Camera.getEntry("tx").getDouble(0D);
		double sign = FunctionsThatShouldBeInTheJDK.getSign(horizontalAngle);
		double absoluteHorizontalAngle = sign * horizontalAngle;
		double absCamAngleRadians = Math.toRadians (absoluteHorizontalAngle);

		if (!(leftPidController.isEnabled() || rightPidController.isEnabled()))  {

			turningLeft = sign > 0D;

			arcLength = ((Math.PI/2) - absCamAngleRadians) * LENGTH_BETWEEN_WHEELS;
			distance = Camera.getDistance() - (LENGTH_BETWEEN_WHEELS* Math.sin(absCamAngleRadians));

			if (turningLeft)
				rightPidController.enable();
			else
				leftPidController.enable();
		}

		if (turningLeft) {
			if (rightTalonSRX.getCurrentPositionInches() > arcLength)
				leftPidController.enable();
		} else {
			if (leftTalonSRX.getCurrentPositionInches() > arcLength)
				rightPidController.enable();
		}

	}
	
	public void drivePID() {

		leftPidController.enable();
		//System.out.println (leftTalonSRX.getCurrentPositionInches());
		//rightPidController.enable();
		//setSpeed(leftTalonSRX.getPIDWrite(), rightTalonSRX.getPIDWrite());
		//setSpeed(leftTalonSRX.getPIDWrite(), 0d/*rightTalonSRX.getPIDWrite()*/);

		//leftTalonSRX.set(leftPidController.get());
		
		
		// System.out.println(leftPID.getPIDSpeed(leftTalonSRX.getCurrentPositionInches()));
		// setSpeed(leftPID.getPIDSpeed(leftTalonSRX.getCurrentPositionInches()), 0);
	
	}

	public void stopDrivePID() {
		leftPidController.disable();
		rightPidController.disable();

	}

	public void setTolerance() {
		leftPidController.setPercentTolerance(TOLERANCE);
		rightPidController.setPercentTolerance(TOLERANCE);
		
	}

	public void setPIDValues(double p, double i, double d, double f) {
		leftPidController.setPID(p, i, d, f);
		//leftPID.setPIDs (p, i, d, f);
		rightPidController.setPID(p, i, d, f);
		//leftPID.setPIDs (p, i, d, f);
	}

	public void setDriveDistance(double setPoint) {
		leftPidController.setSetpoint(leftTalonSRX.inchesToTicks(setPoint));
		//leftPID.setSetPoint(setPoint);
		rightPidController.setSetpoint(rightTalonSRX.inchesToTicks(setPoint));
		//leftPID.setSetPoint(setPoint);
	}


	public double getP () {return leftPidController.getP();}
    public double getI () {return leftPidController.getI();}
	public double getD () {return leftPidController.getD();}
	public double getF () {return leftPidController.getF();}
	public double getSetPoint () {return /*leftPidController.getSetpoint()*/ leftPID.getSetPoint();}
	public double getError () {return /*leftPidController.getError()*/ leftPID.getError();}
	public double getDistanceInchesL() { return leftTalonSRX.getCurrentPositionInches();}
	public double getDistanceTicksL() { return leftTalonSRX.getCurrentPositionTicks();}
	public double getDistanceInchesR() { return rightTalonSRX.getCurrentPositionInches();}
	public double getDistanceTicksR() { return rightTalonSRX.getCurrentPositionTicks();}
	
//myNemChef - Chris
}
