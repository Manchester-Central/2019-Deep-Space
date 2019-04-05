/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;


public class ChaosBetterSpeedController extends SpeedControllerGroup {

    double currentSet;
    double maxAcceleration;

    public ChaosBetterSpeedController (SpeedController arg0, SpeedController arg1, double maxAcceleration) {
        super(arg0, arg1);
        currentSet = 0;
        this.maxAcceleration = maxAcceleration;
    }

    public ChaosBetterSpeedController (SpeedController arg0, SpeedController arg1) {
        super(arg0, arg1);
        currentSet = 0;
        maxAcceleration = 1.0;
    }

    @Override
    public void pidWrite (double speed) {
        if (speed > 0)
            speed = (currentSet + maxAcceleration < speed) ? currentSet + maxAcceleration : speed;
        else
            speed = (currentSet - maxAcceleration > speed) ? currentSet - maxAcceleration : speed;
        set(speed);
        currentSet = speed;
    }

    public double getPIDWrite () {
        return currentSet;
    }

}
