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
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.FunctionsThatShouldBeInTheJDK;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.LinearPot;

/**
 * Add your docs here.
 */
public class Arm {

    private ChaosBetterTalonSRX elbow;
    private WPI_VictorSPX elbow2;
    private ChaosBetterSpeedController elbowGroup;
    private WPI_TalonSRX extender;

    private double angleOffset = 0;
    private static final double ELBOW_SAFE_ANGLE = 15;
    private static final double ARM_HOLD_POWER = 0.19;

    private PIDController elbowPID;
    private PIDController extenderPID;

    private LinearPot elbowPot;
    private LinearPot extenderPot;

    private Wrist wrist;
    private Grabber grab;

    public enum WristMode {intake, tucked, output, straight, nothing};

    public Arm() {

        grab = new Grabber ();
        elbow = new ChaosBetterTalonSRX(PortConstants.ELBOW_JOINT, 0, 0, false);
        elbow2 = new WPI_VictorSPX(PortConstants.ELBOW_2);
        elbowGroup = new ChaosBetterSpeedController(elbow, elbow2, .07);
        extender = new WPI_TalonSRX(PortConstants.EXTENDER);
        wrist = new Wrist();

        elbowPot = new LinearPot(PortConstants.ELBOW_POT, ArmConstants.MIN_ELBOW_VOLTAGE,
                ArmConstants.MAX_ELBOW_VOLTAGE, ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        extenderPot = new LinearPot(PortConstants.EXTENDER_POT, ArmConstants.MIN_EXTENDER_VOLTAGE,
                ArmConstants.MAX_EXTENDER_VOLTAGE, ArmConstants.MIN_EXTENDER_LENGTH, ArmConstants.MAX_EXTENDER_LENGTH);

        elbowPID = new PIDController(ArmConstants.ELBOW_P, ArmConstants.ELBOW_I, ArmConstants.ELBOW_D, elbowPot, elbowGroup);
        extenderPID = new PIDController(ArmConstants.EXTENDER_P, ArmConstants.EXTENDER_I, ArmConstants.EXTENDER_D,
                extenderPot, extender);
        setFeedForward();
        elbowPID.setInputRange(ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        extenderPID.setInputRange(ArmConstants.MIN_EXTENDER_LENGTH, ArmConstants.MAX_EXTENDER_LENGTH);

        Robot.describePID(elbowPID, "elbow pid", elbowPot.getValue(), elbow.get());
        Robot.describePID(extenderPID, "extender pid", extenderPot.getValue(), extender.get());

        extender.setInverted(true);

        
    }

    public void describeElbowPID () {
        Robot.describePID(elbowPID, "elbow pid", elbowPot.getValue(), elbowGroup.getPIDWrite());
    }

    public void describeExtenderPID () {
        Robot.describePID(extenderPID, "extender pid", extenderPot.getValue(), extender.get());
    }

    public void describeWristPID () {
        wrist.describeWristPID();
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
        // elbowPID.setF(ArmConstants.TOTAL_WEIGHT * getCenterOfMass() *Math.cos(Math.toRadians(elbowPot.getValue()) /(ArmConstants.NUMBER_OF_MOTORS
        //         * ArmConstants.GEAR_RATIO * ArmConstants.MOTOR_STALL_TORQUE)));

        elbowPID.setF(Math.cos(Math.toRadians(elbowPot.get())) * ARM_HOLD_POWER );
    }

    public boolean elbowInPosition() {
        return !elbowPID.isEnabled() || FunctionsThatShouldBeInTheJDK.withinPlusOrMinus(getElbowAngle(), elbowPID.getSetpoint(), 0.5);
    }

    public boolean elbowIsSafe() {
        return !elbowPID.isEnabled() || FunctionsThatShouldBeInTheJDK.withinPlusOrMinus(getElbowAngle(), elbowPID.getSetpoint(), ELBOW_SAFE_ANGLE);
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
        //System.out.print (", " + elbowPID.getF() + "\n");
        elbowPID.enable();
    }

    public void setArmToVerticalPosition (double positionInches) {

        double theta = Math.atan2((positionInches - ArmConstants.ARM_HEIGHT_INCHES), ArmConstants.ARM_LENGTH);
        
        double minArmLength = ArmConstants.ARM_LENGTH / Math.cos(theta);

        pidGoToAngle(Math.toDegrees(theta));

        if (elbowInPosition()) {
            setArmDistance(Math.max(ArmConstants.ARM_LENGTH, minArmLength));
        }

        elbowPID.enable();
        //extenderPID.enable();

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

    public double getRawExtender () {
        return extenderPot.get();
    }

    public double getElbowAngle() {
        return elbowPot.getValue();
    }

    public double getRawElbow () {
        return elbowPot.get();
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

    //------------- wrist functions
    
    public void setWristSpeed (double speed) {
        wrist.setSpeed(speed);
    }    

    public void stopWristPID () {
        wrist.stopWristPID();
    }
    
    public void setWristToArmAngle(WristMode mode) {

        double currentElbowAngle = getElbowAngle();
        switch(mode){

            case intake:
                // wrist.setSetPoint(90 - currentElbowAngle);
                wrist.setSetPoint(180 + currentElbowAngle - angleOffset);
                break;

            case tucked:
                wrist.setSetPoint(Wrist.TUCKED_POSITION);
            
                break;
            case output:
                // if (currentElbowAngle > 0) {
                //     wrist.setSetPoint(270 - currentElbowAngle);
                // } else {
                //     wrist.setSetPoint(360 - currentElbowAngle);
                // }
                    wrist.setSetPoint(-currentElbowAngle + angleOffset);

                break;
            case straight:
                // wrist.setSetPoint(Wrist.DEFAULT_ANGLE);
                wrist.setSetPoint(Wrist.TUCKED_POSITION + 90.0);
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

        //  if (elbowIsSafe()) {
            setWristToArmAngle(targetMode);
        // } else {
        //     setWristToArmAngle(WristMode.tucked);
        // }

        wrist.goToSetPoint();

    }

    public double getWristAngle () {
        return wrist.getAngle();
    }

    public double getWirstPotRaw () {
        return wrist.getRawTicks();
    }
    
    //-------------grabber functions
    public void setGrabberSparkSpeed (double speed) {
         grab.setSpark (speed);
    }

   public void extendHatchGrabber () {
      grab.extendHatchGrabber();
   }

   public void retractHatchGrabber () {
      grab.retractHatchGrabber();
   }

   public boolean getGrabberLimitSwitchLeft () {
      return grab.getLimitSwitchLeft();
   }

   public boolean getGrabberLimitSwitchRight () {
      return grab.getLimitSwitchRight();
   }

   public boolean getGrabberBeamSensor () {
      return grab.getBeamSensor();
   }
    
}
