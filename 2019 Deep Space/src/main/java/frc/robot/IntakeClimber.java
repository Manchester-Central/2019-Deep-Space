/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.ChaosSensors.ChaosBetterTalonSRX;

/**
 * Add your docs here.
 */
public class IntakeClimber {

    ChaosBetterTalonSRX rotate0;
    VictorSPX rotate1;
    VictorSPX flywheel;

    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100;
    public static final double TOLERANCE = 0.5;
    public static final double ROTATE_SPEED = 0.1;
    public static final double INTAKE_ANGLE = 2;
    public static final double DOWN_ANGLE = 0;
   // public final double RADIUS = 40;
   // public final double WHEEL_CIRCUMFERENCE_INCHES = 2*Math.PI * RADIUS;

    //private double sign;

    public IntakeClimber () {
        rotate0 = new ChaosBetterTalonSRX(PortConstants.INTAKE_0, 
        2 * Math.PI, ENCODER_TICKS_PER_REVOLUTION, false);
        rotate1 = new VictorSPX(PortConstants.INTAKE_1);
        flywheel = new VictorSPX(PortConstants.FLYWHEEL);
    }

    public void setIntake (double speed) {
        rotate0.set(ControlMode.PercentOutput, speed);
        rotate1.set(ControlMode.PercentOutput, speed);
    
    }

    public void setFlywheel (double speed) {
        flywheel.set(ControlMode.PercentOutput, speed);
    }

    public void setToPosition (double angleInRadians) {

        if (Math.abs( Math.abs(rotate0.getCurrentPositionInches()) - Math.abs(angleInRadians) )
         < TOLERANCE){
            setIntake(0.0);
        } else if (angleInRadians > rotate0.getCurrentPositionInches()) {
            setIntake(ROTATE_SPEED);
        } else {
            setIntake(ROTATE_SPEED);
        }

    }

}
