/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import frc.ChaosSensors.ChaosBetterTalon;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.LinearPot;

/**
 * Add your docs here.
 */
public class Arm {

    ChaosBetterTalonSRX elbow;
    WPI_VictorSPX elbow2;
    WPI_TalonSRX extender;

    LinearPot elbowPot;
    LinearPot extenderPot;

    PIDController elbowPID;
    PIDController extenderPID;
 
    public static final double ELBOW_RADIUS = 4.0;
    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100D;
    private double minElbowVoltage = 0;
    private double maxElbowVoltage = 0;
    private double minElbowAngle = 0;
    private double maxElbowAngle = 0;
    
    private double minExtenderVoltage = 0;
    private double maxExtenderVoltage = 0;
    private double minExtenderLength = 0;
    private double maxExtenderLength = 0;

    public static final double ARM_DISTANCE = 20.0;
    public static final double ARM_WEIGHT = 20.0;
    public static final double EXTENTION_WEIGHT = 20.0;
    public static final double TOTAL_WEIGHT = ARM_WEIGHT;
    public static final double ARM_HEIGHT_INCHES = 42;

    public static final double GEAR_RATIO = 2.0;
    public static final double MOTOR_STALL_TORQUE = 0.71;
    


    private double elbowP = 0.001;
	private double elbowI = 0.4;
    private double elbowD = 0;
    private double extenderP = 0.001;
    private double extenderI = 0;
    private double extenderD = 0;

    public Arm () {
        
        elbow = new ChaosBetterTalonSRX(PortConstants.ELBOW_JOINT, 0, 0, false);
        extender = new WPI_TalonSRX(PortConstants.EXTENDER);

        elbowPot = new LinearPot(PortConstants.Potentiometer, minElbowVoltage, maxElbowVoltage, minElbowAngle, maxElbowAngle);
        extenderPot = new LinearPot(PortConstants.Potentiometer, minExtenderVoltage , maxExtenderVoltage, minExtenderLength, maxExtenderLength);

        elbowPID = new PIDController(elbowP, elbowI, elbowD, elbowPot, elbow);
        extenderPID = new PIDController(extenderP, extenderI, extenderD, extenderPot, extender);
        setFeedForward();
        elbowPID.setInputRange(minElbowAngle, maxElbowAngle);
        extenderPID.setInputRange(minExtenderLength, maxExtenderLength);
    }

    public void setExtenderSpeed (double speed) {
        if ((getExtenderPosition() >= maxExtenderLength) && (speed > 0)) {
            speed = 0;
        }
        else if ((getExtenderPosition() <= minExtenderLength) && (speed < 0)) {
            speed = 0;
        }
        extender.set(speed);
    }

    public void setElbowSpeed (double speed) {
        if ((getElbowAngle() >= maxElbowAngle) && (speed > 0)) {
            speed = 0;
        }
        else if ((getElbowAngle() <= minElbowAngle) && (speed < 0)) {
            speed = 0;
        }
        extender.set(speed);
    }

    public void setFeedForward() {
        elbowPID.setF(TOTAL_WEIGHT * getCenterOfMass() * Math.acos(elbowPot.getValue()) * GEAR_RATIO / MOTOR_STALL_TORQUE);
    }

    public void pidGoToAngle(double angle)  {

        if (!willCrash(angle)) {
            elbowPID.setSetpoint(angle);
            setFeedForward();
            elbowPID.enable();
        }
        else {
            double wantArmDistance = ((ARM_HEIGHT_INCHES / Math.acos(angle)) - 0.5);
            setArmDistance(wantArmDistance);
            elbowPID.disable();
            
        }
    }

    public void setArmDistance (double distance) {
        elbowPID.setSetpoint(distance);
        

    }

    public double getCenterOfMass() {

        return (ARM_WEIGHT * ARM_DISTANCE + EXTENTION_WEIGHT * getExtenderPosition()) / TOTAL_WEIGHT;

    }

    public double getExtenderPosition() {

        return extenderPot.get ;
    
    }

    public double getElbowAngle () {
        return elbowPot.getAngle();
    }

    public boolean willCrash(double angle) {

        // thinko mode
        return (getExtenderPosition() * Math.acos(angle) > ARM_HEIGHT_INCHES);

    }

}
