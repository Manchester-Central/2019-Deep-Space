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
    private double currentSet = 0;

    public ChaosBetterCANSpark(int port) {
        super(port, MotorType.kBrushless);
        setSmartCurrentLimit(50);

    }

    public void setAdjustment (double value) {
        value = FunctionsThatShouldBeInTheJDK.clamp(value, -ADJUSTMENT_MAX, ADJUSTMENT_MAX);
        adjustment = value;
    }

    public double getAdjustment () {
        return adjustment;
    }


    @Override
    public void pidWrite(double output) {
        //System.out.print(adjustment + output);
        //set(output + adjustment);
        currentSet = output;
    }

    public double getPIDWrite() {
        return currentSet;
    }
}
