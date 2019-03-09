/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Controls the pneumatics that lift the robot to climb.
 */
public class PneumaticLift {

    Solenoid pneumatic;

    public PneumaticLift () {

        pneumatic = new Solenoid(PortConstants.CLIMB_SOLENOID);
        pneumatic.set(false);

    }
   
    public void setPositionIn () {
        pneumatic.set(false);

    }

    public void setPositionOut () {
        pneumatic.set(true);

    }

    public boolean isOpen () {
        return pneumatic.get();
    }

}
