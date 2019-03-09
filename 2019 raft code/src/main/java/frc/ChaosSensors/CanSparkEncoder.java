/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ChaosSensors;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class CanSparkEncoder implements PIDSource {

    CANEncoder encoder;
    PIDSourceType type = PIDSourceType.kDisplacement;
    final double circumference;
    final double encoderTicksPerRevolution;

    public CanSparkEncoder (CANEncoder encoder, double circumference, double encoderTicksPerRevolution) {
        
        this.encoder = encoder; 
        this.circumference = circumference;
        this.encoderTicksPerRevolution = encoderTicksPerRevolution;

    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        type = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return type;
    }

    @Override
    public double pidGet() {
        return ticksToInches(encoder.getPosition());
    }
    
    public double inchesToTicks(double inches) {
        return inches * encoderTicksPerRevolution / circumference;
    }

    public double ticksToInches(double ticks) {
		return ticks * circumference / encoderTicksPerRevolution;
    }

    public void reset () {
        encoder.setPosition(0);
        
        
    }

}
