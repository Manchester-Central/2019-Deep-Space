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

import edu.wpi.first.wpilibj.PIDController;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.TalonSRX_Encoder;

/**
 * Add your docs here.
 */
public class IntakeClimber {

    ChaosBetterTalonSRX rotate0;
    TalonSRX_Encoder enc;
    VictorSPX rotate1;
    VictorSPX flywheel;
    PIDController pid;

    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100;
    public static final double ROTATE_SPEED = 0.1;
    public static final double INTAKE_SPEED = 1;
    public static final double INTAKE_ANGLE = 2;
    public static final double DOWN_ANGLE = 0;
    public static final double OUT_ANGLE = Math.PI;
    private static final double P = .0001;
    private static final double I = 0;
    private static final double D = 0;

    public final double RADIUS = 40;
    public final double WHEEL_CIRCUMFERENCE_INCHES = 2*Math.PI * RADIUS;
   // public final double RADIUS = 40;
   // public final double WHEEL_CIRCUMFERENCE_INCHES = 2*Math.PI * RADIUS;

    //private double sign;

    public IntakeClimber () {
        rotate0 = new ChaosBetterTalonSRX(PortConstants.INTAKE_0, 
        2 * Math.PI, ENCODER_TICKS_PER_REVOLUTION, false);
        rotate1 = new VictorSPX(PortConstants.INTAKE_1);
        flywheel = new VictorSPX(PortConstants.FLYWHEEL);
        enc = new TalonSRX_Encoder (rotate0);
        pid = new PIDController(P, I, D, enc, rotate0);
    }

    public void setIntake (double speed) {

        if (getAngleInRadians() >= OUT_ANGLE && speed > 0)
            speed = 0;
        else if (getAngleInRadians() <= DOWN_ANGLE && speed < 0)
            speed = 0;

        rotate0.set(ControlMode.PercentOutput, speed);
        rotate1.set(ControlMode.PercentOutput, speed);
    
    }

    public void goToSetPoint () {
        pid.enable();
    }

    public void stopPIDRotate () {
        pid.disable();
    }

    public void setFlywheel (double speed) {
        flywheel.set(ControlMode.PercentOutput, speed);
    }

    public double getAngleInRadians () {
        return rotate0.getCurrentPositionInches();
    }

    public void setToPosition (double angleInRadians) {

        pid.setSetpoint(angleInRadians);

    }

}
