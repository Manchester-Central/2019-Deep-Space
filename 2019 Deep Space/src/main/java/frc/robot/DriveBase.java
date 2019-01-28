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
	
	
	private double P = 0;
	private double I = 0;
	private double D = 0;
	private double F = 0;
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

		leftEncoder = new TalonSRX_Encoder(leftTalonSRX);
	

		leftPidController = new PIDController(P, I, D, F, leftEncoder, leftTalonSRX);

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

		leftTalonSRX.set(leftSpeed);
		rightTalonSRX.set(rightSpeed);
		

		leftBackVictor.set(leftSpeed);
		leftMidVictor.set(leftSpeed);
		leftFrontVictor.set(leftSpeed);

		rightBackVictor.set(rightSpeed);
<<<<<<< HEAD

=======
		rightMidVictor.set(rightSpeed);
		rightFrontVictor.set(rightSpeed);
		
>>>>>>> 00331ec9f050307cf289a610e4c7a03f4aaae866
    }

	/***
	 * drive based on the camera autonomously
	 */
    public void cameraDrive() {

        double[] speedValues = Camera.getDriveDirections(leftTalonSRX.get(), rightTalonSRX.get());
        setSpeed(speedValues[0] * .1D, speedValues[1] * .1D);

		leftPID.setSetPoint(setPoint);
		
	}

	
	public void drivePID() {

		leftPidController.enable();

		System.out.println(leftPidController.get());
	}

	public void stopDrivePID() {
		leftPidController.disable();
	}

	public void setPIDValues(double p, double i, double d, double f) {
		leftPidController.setPID(p, i, d, f);
	}

	public void setDriveDistance(double setPoint) {
		leftPidController.setSetpoint(setPoint);
	}


	public double getP () {return P;}
    public double getI () {return I;}
	public double getD () {return D;}
	public double getSetPoint () {return leftPidController.getSetpoint();}
    public double getError () {return leftPidController.getError();}
 
//myNemChef - Chris
}
