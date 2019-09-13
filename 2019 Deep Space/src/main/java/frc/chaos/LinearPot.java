/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.chaos;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class LinearPot extends AnalogPotentiometer implements PIDSource {


	private double slope, intercept;
	private PIDSourceType type = PIDSourceType.kDisplacement;

    public LinearPot(int port, double minVoltage, double maxVoltage, double minValue, double maxValue) {

    	super(port);	

		slope = (maxValue - minValue) / (maxVoltage - minVoltage);
		intercept = maxValue - slope * maxVoltage;

    }

	public double getValue() {

		return slope*get() + intercept;
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
		return getValue();
	}


}
