/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Talon;
import frc.FunctionsThatShouldBeInTheJDK;

/**
 * Add your docs here.
 */
public class ChaosBetterTalon extends Talon implements PIDOutput{

    private double sign;
    private double adjustment;
    public final double ADJUSTMENT_MAX = 1;


    double currentSet = 0;
    /***
     * Talon with added functionality
     * @param port - 
     */
    
    public ChaosBetterTalon(int port) {

        super(port);
        adjustment = 1D;
    }

    public void setAdjustment (double value) {
        value = FunctionsThatShouldBeInTheJDK.clamp(value, -ADJUSTMENT_MAX, ADJUSTMENT_MAX);
        adjustment = 1D + value;
    }


    @Override
    public void pidWrite(double output) {
        System.out.print(adjustment * output );
        set(output * adjustment);
    }
  
    public double getPIDWrite() {
        return currentSet;
    }

}