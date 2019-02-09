/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import frc.ChaosSensors.ChaosBetterTalon;

/**
 * Add your docs here.
 */
public class Arm {

    ChaosBetterTalon elbow;
    Victor extender;

    Encoder extenderEncoder;
    AnalogPotentiometer chaosPot;

    PIDController armPID;
 
    public static final double ELBOW_RADIUS = 4.0;
    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100D;

    public static final double ARM_DISTANCE = 20.0;
    public static final double ARM_WEIGHT = 20.0;
    public static final double EXTENTION_WEIGHT = 20.0;
    public static final double TOTAL_WEIGHT = ARM_WEIGHT;

    public static final double GEAR_RATIO = 2.0;
    public static final double MOTOR_STALL_TORQUE = 0.71;
    


    private double P = 0.25;
	private double I = 0.4;
	private double D = 0;
	private double F = 0.5;
	private double setPoint = 24.0;

    public Arm () {
        
        elbow = new ChaosBetterTalon(PortConstants.ELBOW_JOINT);
        extender = new Victor (PortConstants.EXTENDER);

        extenderEncoder = new Encoder(PortConstants.EXTENDER_ENCODER_A , PortConstants.EXTENDER_ENCODER_B);
        chaosPot = new AnalogPotentiometer(PortConstants.Potentiometer);

        armPID = new PIDController(P, I, D, chaosPot, elbow);
    }

    public void setExtenderSpeed (double speed) {
        extender.set(speed);
    }

    public void setElbowSpeed (double speed) {
        extender.set(speed);
    }

    public void pidGoToAngle(double angle)  {

        armPID.setSetpoint(angle);
        armPID.setF(TOTAL_WEIGHT * getCenterOfMass() * Math.acos(getPotValue()) * GEAR_RATIO / MOTOR_STALL_TORQUE);
        armPID.enable();
    }

    public double getCenterOfMass() {

        return (ARM_WEIGHT * ARM_DISTANCE + EXTENTION_WEIGHT * getExtenderPosition()) / TOTAL_WEIGHT;

    }

    public double getExtenderPosition() {

        return (extenderEncoder.get()/ ENCODER_TICKS_PER_REVOLUTION) * (2 * Math.PI) * (ELBOW_RADIUS) ;
    
    }

    public double getPotValue () {
        return chaosPot.get();
    }

}
