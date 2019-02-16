/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ChaosSensors;

import com.revrobotics.CANSparkMax;

import frc.FunctionsThatShouldBeInTheJDK;

/**
 * Add your docs here.
 */
public class ChaosBetterCANSpark extends CANSparkMax{

    private double sign;
    private double adjustment;
    public final double ADJUSTMENT_MAX = 1;

    public ChaosBetterCANSpark(int port) {
        super(port, MotorType.kBrushless);
    }

    public void setAdjustment (double value) {
        value = FunctionsThatShouldBeInTheJDK.clamp(value, -ADJUSTMENT_MAX, ADJUSTMENT_MAX);
        adjustment = 1D + value;
    }


    @Override
    public void pidWrite(double output) {
        System.out.print(adjustment * output);
        set(output * adjustment);
    }
}
