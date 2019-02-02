/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

/**
 * Add your docs here.
 */
public class Arm {

    Talon elbow;
    Victor extender;
    AnalogPotentiometer pot;

    public Arm () {
        
        elbow = new Talon(PortConstants.ELBOW_JOINT);
        extender = new Victor (PortConstants.EXTENDER);
        pot = new AnalogPotentiometer(PortConstants.Potentiometer);
    }

    public void setExtenderSpeed (double speed) {
        extender.set(speed);
    }

    public void setElbowSpeed (double speed) {
        extender.set(speed);
    }

    public double getPotValue () {
        return pot.get();
    }

}
