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

    public Wrist wrist;

    public enum WristMode {intake, tucked, output, straight};

    public Arm() {

        elbow = new ChaosBetterTalonSRX(PortConstants.ELBOW_JOINT, 0, 0, false);
        extender = new WPI_TalonSRX(PortConstants.EXTENDER);
        elbow2 = new WPI_VictorSPX(PortConstants.ELBOW_2);
        wrist = new Wrist();

        elbowPot = new LinearPot(PortConstants.ELBOW_POT, ArmConstants.MIN_ELBOW_VOLTAGE,
                ArmConstants.MAX_ELBOW_VOLTAGE, ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        extenderPot = new LinearPot(PortConstants.EXTENDER_POT, ArmConstants.MIN_EXTENDER_VOLTAGE,
                ArmConstants.MAX_EXTENDER_VOLTAGE, ArmConstants.MIN_EXTENDER_LENGTH, ArmConstants.MAX_EXTENDER_LENGTH);

        elbowPID = new PIDController(ArmConstants.ELBOW_P, ArmConstants.ELBOW_I, ArmConstants.ELBOW_D, elbowPot, elbow);
        extenderPID = new PIDController(ArmConstants.EXTENDER_P, ArmConstants.EXTENDER_I, ArmConstants.EXTENDER_D,
                extenderPot, extender);
        setFeedForward();
        elbowPID.setInputRange(ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        extenderPID.setInputRange(ArmConstants.MIN_EXTENDER_LENGTH, ArmConstants.MAX_EXTENDER_LENGTH);

        extender.setInverted(true);
    }

    public void enableExtenderPID () {
        extenderPID.enable();
    }

    public void enableElbowPID () {
        elbowPID.enable();
    }

    public void disableExtenderPID () {
        extenderPID.disable();
    }

    public void disableElbowPID () {
        elbowPID.disable();
    }

    /***
     * Controls speed of extender with restriction to game rules
     * @param speed
     */
    public void setExtenderSpeed(double speed) {
        // if ((getExtenderPosition() >= ArmConstants.MAX_EXTENDER_LENGTH) && (speed > 0)) {
        //     speed = 0;
        // } else if ((getExtenderPosition() <= ArmConstants.MIN_EXTENDER_LENGTH) && (speed < 0)) {
        //     speed = 0;
        // } else if (outsideReach(getExtenderPosition(), getElbowAngle()) && (speed > 0)) {
        //     speed = 0;
        // }
        // extender.set(speed - (1.0/6.0));
    
        double adjustedSpeed = (FunctionsThatShouldBeInTheJDK.withinPlusOrMinus(speed, 0, 0.1)) ? speed -/*- Math.sin(getElbowAngle()*/ (0.18) : speed;
        extender.set(adjustedSpeed);
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
     * https://www.chiefdelphi.com/t/velocity-limiting-pid/164908/22
     */
    public void setFeedForward() {
        elbowPID.setF(ArmConstants.TOTAL_WEIGHT * getCenterOfMass() / (Math.cos(Math.toRadians(elbowPot.getValue())
                * ArmConstants.GEAR_RATIO * ArmConstants.MOTOR_STALL_TORQUE)));
    }

    public boolean elbowInPosition() {
        return !elbowPID.isEnabled() || FunctionsThatShouldBeInTheJDK.withinPlusOrMinus(getElbowAngle(), elbowPID.getSetpoint(), 0.5);
    }

    /***
     * Set arm to angle with PID, will avoid going outside frame perimeter of colliding with robot
     * @param angle - 0 is parallel to ground, positive = up, negative = down
     */
    public void pidGoToAngle(double angle) {
        // REMEMBER @TODO
        //setArmDistance(0);
        setFeedForward();

        // if (willCrash(angle) || outsideReach(extenderPot.getValue(), angle)) {

        //     elbowPID.setSetpoint(getElbowAngle());
            
        // } else {

            elbowPID.setSetpoint(angle);
        // }
        System.out.print (", " + elbowPID.getF() + "\n");
        elbowPID.enable();
    }

    public void setArmToVerticalPosition (double positionInches) {

        double theta = Math.atan2((positionInches - ArmConstants.ARM_HEIGHT_INCHES), ArmConstants.ARM_LENGTH);
        //z = x / cos
        double minArmLength = ArmConstants.ARM_LENGTH / Math.cos(theta);

        pidGoToAngle(Math.toDegrees(theta));

        if (elbowInPosition()) {
            setArmDistance(Math.max(ArmConstants.ARM_LENGTH, minArmLength));
        }

        elbowPID.enable();
        extenderPID.enable();

    }

    public void setArmDistance(double distance) {
        extenderPID.setSetpoint(distance);

    }
    

    public double getCenterOfMass() {

        return ((ArmConstants.ARM_WEIGHT * ArmConstants.ARM_CENTER_OF_MASS
            + ArmConstants.EXTENDER_ARM_WEIGHT * (ArmConstants.EXTENSION_TOTAL_CENTER_OF_MASS + getExtenderPosition())) 
            / (ArmConstants.TOTAL_WEIGHT));

    }

    public double getExtenderPosition() {
        return extenderPot.getValue();

    }

    public double getElbowAngle() {
        return elbowPot.getValue();
    }

    public boolean willCrash(double angle) {

        // thinko mode
        return getExtenderPosition() * Math.acos(angle) > ArmConstants.ARM_HEIGHT_INCHES;

    }

    public double armLength(double extenderLength) {
        return ArmConstants.ARM_LENGTH + extenderLength;
    }

    

    /***
     * Most the extender can be extended legally at an angle
     * @param angle
     * @return
     */
    public double maxExtenderLength(double angle) {
        return (ArmConstants.MAX_REACH_X / Math.acos(angle)) - ArmConstants.ARM_LENGTH;
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
    
    public void setWristToArmAngle(WristMode mode) {

        double currentElbowAngle = getElbowAngle();
        switch(mode){

            case intake:
                wrist.setSetPoint(90 - currentElbowAngle);

                break;

            case tucked:
                wrist.setSetPoint(Wrist.TUCKED_POSITION);
            
                break;
            case output:
                if (currentElbowAngle > 0) {
                    wrist.setSetPoint(270 - currentElbowAngle);
                } else {
                    wrist.setSetPoint(360 - currentElbowAngle);
                }
                break;
            case straight:
                wrist.setSetPoint(Wrist.DEFAULT_ANGLE);
                break;
            default:
                wrist.setSetPoint(wrist.getAngle());
                break;
        }
    }


    /**
     * 
     * @param targetMode - The mode that the wrist will be set to once arm is in position,
     * either intake or output
     */
    public void autoMoveWrist(WristMode targetMode) {

        if (elbowInPosition()) {
            setWristToArmAngle(targetMode);
        } else {
            setWristToArmAngle(WristMode.tucked);
        }

        wrist.goToSetPoint();
    }
}
