/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static org.junit.Assume.assumeNoException;

import edu.wpi.first.wpilibj.Victor;

/**
 * Add your docs here.
 */
public class IntakeClimber {

    Victor intake0;
    Victor intake1;
    Victor flywheel;

    public IntakeClimber () {
        intake0 = new Victor(PortConstants.INTAKE_0);
        intake1 = new Victor(PortConstants.INTAKE_1);
        flywheel = new Victor(PortConstants.FLYWHEEL);
    }

    public void setIntake (double speed) {
        intake0.set(speed);
        intake1.set(speed);
    
    }

    public void setFlywheel (double speed) {
        flywheel.set(speed);
    }

}
