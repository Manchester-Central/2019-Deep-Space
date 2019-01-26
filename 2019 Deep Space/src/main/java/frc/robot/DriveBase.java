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

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
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
	
	WPI_TalonSRX rightTalonSRX;
    WPI_TalonSRX leftTalonSRX;

    public DriveBase() {

       // leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        //rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
       
        //leftPID = new KYSPID(P, I, D, setPoint)
        //pid = new PIDLinked(leftPID, rightPID);
        

        // above is real code, below is raft, comment/uncomment to make work
		
		rightTalonSRX = new WPI_TalonSRX(PortConstants.RIGHT_CAN_TALON);
		leftTalonSRX = new WPI_TalonSRX(PortConstants.LEFT_CAN_TALON);

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

    // set speed
    public void setSpeed (double leftSpeed, double rightSpeed) {
        leftTalonSRX.set(leftSpeed);
		rightTalonSRX.set(rightSpeed);
		
		leftFrontVictor.set(leftSpeed);
		leftMidVictor.set(leftSpeed);
		leftBackVictor.set(leftSpeed);

		rightFrontVictor.set(rightSpeed);
		rightMidVictor.set(rightSpeed);
		rightBackVictor.set(rightSpeed);
    }

    public void cameraDrive() {

        double[] speedValues = Camera.getDriveDirections(leftTalonSRX.get(), rightTalonSRX.get());
        setSpeed(speedValues[0] * .1D, speedValues[1] * .1D);

    }

    




//myNemChef - Chris
}
