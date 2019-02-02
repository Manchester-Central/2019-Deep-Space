/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Camera;

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
	
	private double P = 0;
	private double I = 0;
	private double D = 0;
	private double setPoint = 0;

	public static final double ENCODER_TICKS_PER_REVOLUTION = 0;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = 0;


    public DriveBase() {

       // leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        //rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
       
        leftPID = new KYSPID(P, I, D, setPoint);
		//pid = new PIDLinked(leftPID, rightPID);
		
	

        // above is real code, below is raft, comment/uncomment to make work
		
		rightTalonSRX = new ChaosBetterTalonSRX(PortConstants.RIGHT_CAN_TALON, WHEEL_CIRCUMFERENCE_INCHES, ENCODER_TICKS_PER_REVOLUTION);
		leftTalonSRX = new ChaosBetterTalonSRX(PortConstants.LEFT_CAN_TALON, WHEEL_CIRCUMFERENCE_INCHES, ENCODER_TICKS_PER_REVOLUTION);

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
		

	}

	public void resetEncoders () {
		rightTalonSRX.resetEncoder();
		leftTalonSRX.resetEncoder();
	}
	

    // set speed
    public void setSpeed (double leftSpeed, double rightSpeed) {
        leftTalonSRX.set(leftSpeed);
		rightTalonSRX.set(rightSpeed);
		

		leftBackVictor.set(leftSpeed);
		leftMidVictor.set(leftSpeed);
		leftFrontVictor.set(leftSpeed);

		rightBackVictor.set(rightSpeed);
		rightMidVictor.set(rightSpeed);
		rightFrontVictor.set(rightSpeed);
		
    }

    public void cameraDrive() {

        double[] speedValues = Camera.getDriveDirections(leftTalonSRX.get(), rightTalonSRX.get());
        setSpeed(speedValues[0] * .1D, speedValues[1] * .1D);

		leftPID.setSetPoint(setPoint);
		
	}

	
	public void drivePID() {

		setSpeed(leftPID.getPIDSpeed(leftTalonSRX.getCurrentPositionInches()), 0);

		SmartDashboard.putNumber("Current Set Speed", leftPID.getPIDSpeed(leftTalonSRX.getCurrentPositionInches()));
	}

	public void setPIDValues(double p, double i, double d) {
		leftPID.setPIDs(P, I, D);
	}

	public void setDriveDistance(double setPoint) {
		leftPID.setSetPoint(setPoint);
	}


	public double getP () {return P;}
    public double getI () {return I;}
    public double getD () {return D;}
    public double getError () {return leftPID.getError();}
    public double getErrorSum () {return leftPID.getErrorSum();}
    public double getSetPoint () {return leftPID.getSetPoint();}
    public double getPreviousError () {return leftPID.getPreviousError();}
//myNemChef - Chris
}
