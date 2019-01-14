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

    public Talon leftFront;
    public Talon rightFront;


    public DriveBase() {

        // leftFront = new CANSparkMax(PortConstants.LEFT_FRONT_SPARK, MotorType.kBrushless );
        // rightFront = new CANSparkMax(PortConstants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
        leftFront = new Talon(PortConstants.LEFT_FRONT_SPARK);
        rightFront = new Talon(PortConstants.RIGHT_FRONT_SPARK);
        
    }

    // set speed
    public void setSpeed (double leftSpeed, double rightSpeed) {
        leftFront.set(leftSpeed);
        rightFront.set(rightSpeed);
    }


    




}
