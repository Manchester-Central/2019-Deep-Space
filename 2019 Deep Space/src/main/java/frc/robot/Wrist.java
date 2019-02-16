/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.TalonSRX_Encoder;

/**
 * Add your docs here.
 */
public class Wrist {

   ChaosBetterTalonSRX speedController;
   TalonSRX_Encoder speedControllerEncoder;
   PIDController pid;

   private static final double CIRCUMFERENCE = 1;
   private static final double ENCODER_TICKS_PER_REVOLUTION = 4100;
   private static final double maxAngle = 180;
   private static final double minAngle = 0;
   private static final double P = 0;
   private static final double I = 0;
   private static final double D = 0;


   public Wrist() {

      speedController = new ChaosBetterTalonSRX(PortConstants.WRIST, CIRCUMFERENCE, ENCODER_TICKS_PER_REVOLUTION,
            false);
         
      speedControllerEncoder = new TalonSRX_Encoder(speedController);
      pid = new PIDController(P, I, D, speedControllerEncoder, speedController);
   }

   public void setSpeed(double speed) {
      if ((getAngle() >= maxAngle) && (speed > 0)) {
         speed = 0;
      } else if ((getAngle() <= minAngle) && (speed < 0)) {
         speed = 0;
      }
      speedController.set(speed);
   }

   public double getRawTicks() {
      return speedController.getCurrentPositionTicks();
   }

   public double getAngle() {
      return speedController.getEncoderAngle();
   }

   public void setSetPoint(double setPoint) {
      pid.setSetpoint(setPoint);
   }

   public void goToSetPoint() {
      pid.enable();
   }


}
