/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import frc.Camera;

/**
 * Calculates new speed
 */
public class DriveBase {

    public CANSparkMax leftFront;
    public CANSparkMax rightFront;

    private KYSPID leftPID;
    private KYSPID rightPID;
    private PIDLinked pid;


    public DriveBase() {

        leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
       
        //leftPID = new KYSPID(P, I, D, setPoint)
        //pid = new PIDLinked(leftPID, rightPID);
        
    }

    // set speed
    public void setSpeed (double leftSpeed, double rightSpeed) {
        leftFront.set(leftSpeed);
        rightFront.set(-rightSpeed);
    }

    public void cameraDrive() {

        double[] speedValues = Camera.getDriveDirections(leftFront.get() ,rightFront.get());
        setSpeed(speedValues[0], speedValues[1]);

    }

    




//myNemChef - Chris
}
