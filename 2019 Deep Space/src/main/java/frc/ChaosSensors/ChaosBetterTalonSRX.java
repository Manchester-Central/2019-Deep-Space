/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ChaosSensors;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.FunctionsThatShouldBeInTheJDK;

/**
 * Add your docs here.
 */
public class ChaosBetterTalonSRX extends WPI_TalonSRX implements PIDOutput{

    public final double ENCODER_TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE_INCHES;
    private double sign;
    private double adjustment;
    public final double ADJUSTMENT_MAX = 1;


    double currentSet = 0;
    /***
     * WPI TalonSRX with added functionality
     * @param port - 
     * @param circumference - 
     * @param encoderTicksPerRevolution
     */
    
    public ChaosBetterTalonSRX(int port, double circumference, double encoderTicksPerRevolution, boolean isInverted) {

        super(port);
        adjustment = 1D;
        WHEEL_CIRCUMFERENCE_INCHES = circumference;
        ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
        sign = (isInverted) ? -1 : 1;
    }

    public void setAdjustment (double value) {
        value = FunctionsThatShouldBeInTheJDK.clamp(value, -ADJUSTMENT_MAX, ADJUSTMENT_MAX);
        adjustment = 1D + value;
    }

    public void resetEncoder() {   
        getSensorCollection().setQuadraturePosition(0,10);
    }


    public double getCurrentPositionTicks() {

        return sign * getSensorCollection().getQuadraturePosition();

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

    @Override
    public void pidWrite(double output) {
        System.out.print(adjustment * output + ", ");
        set(ControlMode.PercentOutput, output * adjustment);
    }
  
    public double getPIDWrite() {
        return currentSet;
    }

    public double getEncoderAngle() {
        return ((getCurrentPositionTicks() / ENCODER_TICKS_PER_REVOLUTION) *  360) % 360;
    }


}
