/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PIDController;
import frc.FunctionsThatShouldBeInTheJDK;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.LinearPot;

/**
 * Add your docs here.
 */
public class Arm {

    ChaosBetterTalonSRX elbow;
    WPI_VictorSPX elbow2;
    WPI_TalonSRX extender;

    PIDController elbowPID;
    PIDController extenderPID;

    LinearPot elbowPot;
    LinearPot extenderPot;

    Wrist wrist;

    public Arm() {

        elbow = new ChaosBetterTalonSRX(PortConstants.ELBOW_JOINT, 0, 0, false);
        extender = new WPI_TalonSRX(PortConstants.EXTENDER);
        wrist = new Wrist();

        elbowPot = new LinearPot(PortConstants.Potentiometer, ArmConstants.MIN_ELBOW_VOLTAGE,
                ArmConstants.MAX_ELBOW_VOLTAGE, ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        extenderPot = new LinearPot(PortConstants.Potentiometer, ArmConstants.MIN_EXTENDER_VOLTAGE,
                ArmConstants.MAX_EXTENDER_VOLTAGE, ArmConstants.MIN_EXTENDER_LENGTH, ArmConstants.MAX_EXTENDER_LENGTH);

        elbowPID = new PIDController(ArmConstants.ELBOW_P, ArmConstants.ELBOW_I, ArmConstants.ELBOW_D, elbowPot, elbow);
        extenderPID = new PIDController(ArmConstants.EXTENDER_P, ArmConstants.EXTENDER_I, ArmConstants.EXTENDER_D,
                extenderPot, extender);
        setFeedForward();
        elbowPID.setInputRange(ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        extenderPID.setInputRange(ArmConstants.MIN_EXTENDER_LENGTH, ArmConstants.MAX_EXTENDER_LENGTH);
    }

    /***
     * Controls speed of extender with restriction to game rules
     * @param speed
     */
    public void setExtenderSpeed(double speed) {
        if ((getExtenderPosition() >= ArmConstants.MAX_EXTENDER_LENGTH) && (speed > 0)) {
            speed = 0;
        } else if ((getExtenderPosition() <= ArmConstants.MIN_EXTENDER_LENGTH) && (speed < 0)) {
            speed = 0;
        } else if (outsideReach(getExtenderPosition(), getElbowAngle()) && (speed > 0)) {
            speed = 0;
        }
        extender.set(speed);
    }

    /***
     * Controls speed of angle (elbow) with restriction to game rules
     * @param speed
     */
    public void setElbowSpeed(double speed) {
        double angle = getElbowAngle();

        if ((angle >= ArmConstants.MAX_ELBOW_ANGLE) && (speed > 0)) {
            speed = 0;
        } else if ((angle <= ArmConstants.MIN_ELBOW_ANGLE) && (speed < 0)) {
            speed = 0;
        } else if (outsideReach(getExtenderPosition(), angle)) {
            // If angle and speed are going in the same direction then the arm moves away
            // from the penalty zone
            if (FunctionsThatShouldBeInTheJDK.getSign(angle) != FunctionsThatShouldBeInTheJDK.getSign(speed)) {
                speed = 0;
            }
        }
        elbow.set(speed);
    }

    /***
     * Sets the speed of the arm to oppose gravity
     */
    public void setFeedForward() {
        elbowPID.setF(ArmConstants.TOTAL_WEIGHT * getCenterOfMass() * Math.acos(elbowPot.getValue())
                * ArmConstants.GEAR_RATIO / ArmConstants.MOTOR_STALL_TORQUE);
    }

    /***
     * Set arm to angle with PID, will avoid going outside frame perimeter of colliding with robot
     * @param angle - 0 is parallel to ground, positive = up, negative = down
     */
    public void pidGoToAngle(double angle) {

        if (willCrash(angle)) {

            double wantArmDistance = ((ArmConstants.ARM_HEIGHT_INCHES / Math.acos(angle)) - 0.5);
            setArmDistance(wantArmDistance);
            elbowPID.disable();

        } else if (outsideReach(extenderPot.getValue(), angle)) {

            setArmDistance(maxExtenderLength(angle) - 0.5);
            elbowPID.disable();
            
        } else {

            elbowPID.setSetpoint(angle);
            setFeedForward();
            elbowPID.enable();
        }
    }

    public void setArmDistance(double distance) {
        extenderPID.setSetpoint(distance);

    }

    public double getCenterOfMass() {

        return (ArmConstants.ARM_WEIGHT * ArmConstants.ARM_DISTANCE
                + ArmConstants.EXTENSION_WEIGHT * getExtenderPosition()) / ArmConstants.TOTAL_WEIGHT;

    }

    public double getExtenderPosition() {
        return extenderPot.getValue();

    }

    public double getElbowAngle() {
        return elbowPot.getValue();
    }

    public boolean willCrash(double angle) {

        // thinko mode
        return (getExtenderPosition() * Math.acos(angle) > ArmConstants.ARM_HEIGHT_INCHES);

    }

    public double armLength(double extenderLength) {
        return ArmConstants.ARM_DISTANCE + extenderLength;
    }

    /***
     * How far out the horizontal component of the arm extension
     * @param extenderLength
     * @param angle
     * @return
     */
    public double armDistanceX(double extenderLength, double angle) {
        // thinko mode
        return armLength(extenderLength) * Math.acos(angle);
    }

    /***
     * Most the extender can be extended legally at an angle
     * @param angle
     * @return
     */
    public double maxExtenderLength(double angle) {
        return (ArmConstants.MAX_REACH_X / Math.acos(angle)) - ArmConstants.ARM_DISTANCE;
    }

    /***
     * True if arm will break rules
     * @param extenderLength
     * @param angle
     * @return
     */
    public boolean outsideReach(double extenderLength, double angle) {
        return (extenderLength > maxExtenderLength(angle));
    }
    
    /**
     * Keeps the wrist at an angle from horizontal when the arm moves
     * @param angle - the angle from horizontal, positive is up 
     */
    public void keepWristAtAngle (double angle) {
        wrist.setSetPoint(Math.abs(getElbowAngle()) + (angle * -FunctionsThatShouldBeInTheJDK.getSign(getElbowAngle())));
    }
}
