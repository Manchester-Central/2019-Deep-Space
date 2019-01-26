

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class Robot extends IterativeRobot {
  


  DriveBase drive;
  ControllerSecretary cs;
  

  /**
   * 
   */
  @Override
  public void robotInit() {
    
    drive = new DriveBase();
    cs = new ControllerSecretary();

    
  }

  /**
   * 
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Current Drive Target", drive.getSetPoint());
  }

  /**
   * 
   */
  @Override
  public void autonomousInit() {



  }

  /**
   * 
   */
  @Override
  public void autonomousPeriodic() {
    


  }

  /**
   * 
   */
  @Override
  public void teleopPeriodic() {

    
    if (cs.driver.buttonPressed(Controller.DOWN_A)) {
      drive.cameraDrive();
    } else if (cs.driver.buttonPressed(Controller.LEFT_X)) {
      drive.drivePID();
    } else {
      drive.setSpeed(cs.driver.getLeftY(), cs.driver.getRightY());
    }
    

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    

  }
  @Override
  public void disabledPeriodic() {

    drive.resetEncoders();

    drive.setPIDValues(SmartDashboard.getNumber("p-value", 0), SmartDashboard.getNumber("i-value", 0),
     SmartDashboard.getNumber("d-value", 0));

     drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 0));
  }


}
