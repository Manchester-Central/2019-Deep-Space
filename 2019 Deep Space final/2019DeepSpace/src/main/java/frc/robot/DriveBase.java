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

/**
 * Add your docs here.
 */
public class DriveBase {

    public Talon leftMid;
    public Talon rightMid;

    public Talon leftBack;
    public Talon rightBack;

    public Talon leftFront;
    public Talon rightFront;


    public DriveBase() {

        // leftFront = new CANSp arkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        // rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
        leftMid = new Talon(PortConstants.LEFT_MID_SPARK);
        rightMid = new Talon(PortConstants.RIGHT_MID_SPARK);

        leftBack = new Talon(PortConstants.LEFT_BACK_SPARK);
        rightBack = new Talon(PortConstants.RIGHT_BACK_SPARK);

        leftFront = new Talon(PortConstants.LEFT_FRONT_SPARK);
        rightFront = new Talon(PortConstants.RIGHT_FRONT_SPARK);
        
    }

    public void moveThing (double x) {
        leftMid.set(x);
    }

    // set speed
    public void setSpeed1 (double leftSpeed, double rightSpeed) {
        leftFront.set(leftSpeed);
        rightFront.set(-rightSpeed);
    }


     // set speed
     public void setSpeed2 (double leftSpeed, double rightSpeed) {
        leftMid.set(leftSpeed);
        rightMid.set(-rightSpeed);
    }


     // set speed
     public void setSpeed3 (double leftSpeed, double rightSpeed) {
        leftBack.set(leftSpeed);
        rightBack.set(-rightSpeed);
    }


    




}
