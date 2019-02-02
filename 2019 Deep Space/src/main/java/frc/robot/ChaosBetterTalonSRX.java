/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Add your docs here.
 */
public class ChaosBetterTalonSRX extends WPI_TalonSRX implements PIDOutput{

    public final double ENCODER_TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE_INCHES;

    /***
     * WPI TalonSRX with added functionality
     * @param port - 
     * @param circumference - 
     * @param encoderTicksPerRevolution
     */
    public ChaosBetterTalonSRX(int port, double circumference, double encoderTicksPerRevolution) {

        super(port);
        WHEEL_CIRCUMFERENCE_INCHES = circumference;
        ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
    }

    public void resetEncoder() {

        setSelectedSensorPosition(0, 0, 0);    
        
    }


    public double getCurrentPositionTicks() {

        return getSelectedSensorPosition(0);

    }

    public double getCurrentPositionInches() {
        return ticksToInches(getCurrentPositionTicks());
    }

    

    public double inchesToTicks(double inches) {
		
		return inches * ENCODER_TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_INCHES;
		
	}

	public  double ticksToInches(double ticks) {
		
		return ticks * WHEEL_CIRCUMFERENCE_INCHES / ENCODER_TICKS_PER_REVOLUTION;
		
    }
    
    

   

}
