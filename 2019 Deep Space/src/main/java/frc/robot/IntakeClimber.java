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
import frc.chaos.ChaosBetterSpeedController;
import frc.chaos.ChaosBetterTalonSRX;
import frc.chaos.LinearPot;

public class IntakeClimber {

    ChaosBetterTalonSRX rotate0;
    WPI_VictorSPX rotate1;
    WPI_VictorSPX flywheel;
    PIDController pid;
    LinearPot anglePot;

    /*              (The 0 angle may appear calibrated slightly below level
     *        90     with the floor because of the bend in the arm itself.
     *         |     It's also zero-ed at the movment limit, not true level.)
     *         |
     *   0 ----+---- 180
     *                                ____
     *                                | \ \
     *                                |  \ \
     *              (4)               |  |\ \
     *                 (5)            |  | \ \
     *                    /O/         |  |  \ \
     *                   / /          |  |   \ \    |
     *     (3)          / /           |  |    \ \   |
     *                 / /            |  |     \ \  |
     *                / /    (6)      |  |    +------+
     *               / /      (2)     |  |    |      |<
     *               +--------------------+   +------+
     *  (1)          |                    |
     *               +--------------------+
     *                  U              U
     *
     * 1: "out" position, the maximum the climb-take can be rotated out
     * 2: "in" position, the maximum the climb-take can be folded in
     * 3: "intake" position for pulling in a "cargo" gamepeice
     * 4: "vertical" position is used to keep the climb-take within the robot
     *    frame permiter while avoiding contact with the arm
     * 5,6: "MAX_SAFE" and "MIN_SAFE", between these positions the climb-take
     *      is located within the arm's normal range of motion
     */

    // Known positions
    public static final double VERTICAL_POSITION = 115.0;
    public static final double INTAKE_ANGLE = 55.0;
    public static final double IN_ANGLE = 205.0;
    public static final double CLIMBTAKE_MAX_SAFE_ANGLE = VERTICAL_POSITION + 5.0;
    public static final double CLIMBTAKE_MIN_SAFE_ANGLE = IN_ANGLE - 5.0;
    public static final double MAX_LEGAL_ANGLE = VERTICAL_POSITION;
    public static final double OUT_ANGLE = 0.0;

    // Calibration positions and potetiometer readings
    public static final double CAL_A_ANGLE = 0.0;
    public static final double CAL_B_ANGLE = VERTICAL_POSITION;
    public static final double CAL_A_VOLTAGE = 0.1057;
    public static final double CAL_B_VOLTAGE = 0.338;


    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100;
    public static final double INTAKE_SPEED = -1;
    public static final double CLIMB_SPEED = -0.3;
    public static final double P = .07;
    public static final double I = 0;
    public static final double D = 0;

    public IntakeClimber () {

        rotate0 = new ChaosBetterTalonSRX(PortConstants.INTAKE_TALON,
            2 * Math.PI, ENCODER_TICKS_PER_REVOLUTION, false);
        rotate1 = new WPI_VictorSPX(PortConstants.INTAKE_VICTOR);

        ChaosBetterSpeedController group = new ChaosBetterSpeedController(rotate0, rotate1, 20);
        anglePot = new LinearPot(PortConstants.CLIMBER_POT, CAL_A_VOLTAGE, CAL_B_VOLTAGE, CAL_A_ANGLE, CAL_B_ANGLE);

        pid = new PIDController(P, I, D, anglePot, group);

        flywheel = new WPI_VictorSPX(PortConstants.FLYWHEEL);

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
        boolean belowSafety = (getAngle() <= IntakeClimber.CLIMBTAKE_MAX_SAFE_ANGLE)
            && (getTargetAngle() > CLIMBTAKE_MAX_SAFE_ANGLE);
        boolean aboveSafety = (getAngle() >= IntakeClimber.CLIMBTAKE_MIN_SAFE_ANGLE)
            && (getTargetAngle() < CLIMBTAKE_MIN_SAFE_ANGLE);
        boolean inDangerZone = (getAngle() >= IntakeClimber.CLIMBTAKE_MIN_SAFE_ANGLE)
            && (getAngle() <= IntakeClimber.CLIMBTAKE_MAX_SAFE_ANGLE);

        return (belowSafety) ||  (aboveSafety) ||  (inDangerZone);
    }

    public double getTargetAngle() {
        return pid.getSetpoint();
    }

    public double getRawRotateValue () {
        return anglePot.get();
    }

}
