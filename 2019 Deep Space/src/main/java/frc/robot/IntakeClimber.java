/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PIDController;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.LinearPot;

/**
 * Add your docs here.
 */
public class IntakeClimber {

    ChaosBetterTalonSRX rotate0;
    WPI_VictorSPX rotate1;
    WPI_VictorSPX flywheel;
    PIDController pid;
    LinearPot anglePot;

    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100;
    public static final double ROTATE_SPEED = 1.0;
    public static final double INTAKE_SPEED = -1;
    public static final double VERTICAL_POSITION = 115.0;
    public static final double INTAKE_ANGLE = 55.0;
    public static final double IN_ANGLE = 205.0;
    public static final double OUT_ANGLE = 0.0;
    private static final double P = .07;
    private static final double I = 0;
    private static final double D = 0;
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 204.0;
    public static final double MIN_VOLTAGE = 0.04;
    public static final double MAX_VOLTAGE = 0.62;
    public static final double CLIMBTAKE_MAX_SAFE_ANGLE = VERTICAL_POSITION + 5.0;
    public static final double CLIMBTAKE_MIN_SAFE_ANGLE = IN_ANGLE - 5.0;
    public static final double MAX_LEGAL_ANGLE = VERTICAL_POSITION;
    public final double RADIUS = 40;
    public final double WHEEL_CIRCUMFERENCE_INCHES = 2*Math.PI * RADIUS;
   // public final double RADIUS = 40;
   // public final double WHEEL_CIRCUMFERENCE_INCHES = 2*Math.PI * RADIUS;

    //private double sign;

    public IntakeClimber () {
        rotate0 = new ChaosBetterTalonSRX(PortConstants.INTAKE_0, 
        2 * Math.PI, ENCODER_TICKS_PER_REVOLUTION, false);
        rotate1 = new WPI_VictorSPX(PortConstants.INTAKE_1);
        ChaosBetterSpeedController group = new ChaosBetterSpeedController(rotate0, rotate1, 20);
        flywheel = new WPI_VictorSPX(PortConstants.FLYWHEEL);
        anglePot = new LinearPot(PortConstants.CLIMBER_POT, MIN_VOLTAGE, MAX_VOLTAGE, MIN_ANGLE, MAX_ANGLE);
        pid = new PIDController(P, I, D, anglePot, group);
        Robot.describePID(pid, "intake pid", anglePot.getValue(), rotate0.get());
    }

    public void describeClimberPID () {
        Robot.describePID(pid, "intake pid", anglePot.getValue(), rotate0.getPIDWrite());
    }

    public void setRotateSpeed (double speed) {

        if (getAngle() <= OUT_ANGLE && speed > 0)
            speed = 0;
        else if (getAngle() >= MAX_LEGAL_ANGLE && speed < 0)
            speed = 0;

        rotate0.set(ControlMode.PercentOutput, -speed);
        rotate1.set(ControlMode.PercentOutput, -speed);
    
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

    public double getAngle () {
        return anglePot.getValue();
    }

    public void setToPosition (double angle) {

        pid.setSetpoint(angle);

    }

    public boolean isClimbtakeUnsafe() {
        boolean belowSafety = (getAngle() <= IntakeClimber.CLIMBTAKE_MAX_SAFE_ANGLE) && (getTargetAngle() > CLIMBTAKE_MAX_SAFE_ANGLE);
        boolean aboveSafety = (getAngle() >= IntakeClimber.CLIMBTAKE_MIN_SAFE_ANGLE) && (getTargetAngle() < CLIMBTAKE_MIN_SAFE_ANGLE);
        boolean inDangerZone = (getAngle() >= IntakeClimber.CLIMBTAKE_MIN_SAFE_ANGLE) && (getAngle() <= IntakeClimber.CLIMBTAKE_MAX_SAFE_ANGLE);

        return (belowSafety) ||  (aboveSafety) ||  (inDangerZone);
    }

    public double getTargetAngle() {
        return pid.getSetpoint();
    }

    public double getRawRotateValue () {
        return anglePot.get();
    }

}
