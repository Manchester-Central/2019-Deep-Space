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

/**
 * Add your docs here.
 */
public class IntakeClimber {

    TalonSRX intake0;
    VictorSPX intake1;
    VictorSPX flywheel;

    public final double ENCODER_TICKS_PER_REVOLUTION = 4100;
    public final double RADIUS = 40;
    public final double WHEEL_CIRCUMFERENCE_INCHES = 2*Math.PI * RADIUS;
    private double sign;

    public IntakeClimber () {
        intake0 = new TalonSRX(PortConstants.INTAKE_0);
        intake1 = new VictorSPX(PortConstants.INTAKE_1);
        flywheel = new VictorSPX(PortConstants.FLYWHEEL);
    }

    public void setIntake (double speed) {
        intake0.set(ControlMode.PercentOutput, speed);
        intake1.set(ControlMode.PercentOutput, speed);
    
    }

    public void setFlywheel (double speed) {
        flywheel.set(ControlMode.PercentOutput, speed);
    }

}
