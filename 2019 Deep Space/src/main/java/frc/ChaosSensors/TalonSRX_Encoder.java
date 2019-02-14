/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ChaosSensors;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class TalonSRX_Encoder implements PIDSource {

    private ChaosBetterTalonSRX check;
    private PIDSourceType type = PIDSourceType.kDisplacement;

    public TalonSRX_Encoder(ChaosBetterTalonSRX check) {

        this.check = check;


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
		return check.getCurrentPositionTicks();
	}


}
