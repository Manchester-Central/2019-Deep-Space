

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

    drive.setSpeed1(-cs.driver.getLeftY(), -cs.driver.getRightY());

    drive.setSpeed2(-cs.operator1.getLeftY(), -cs.operator1.getRightY());

    drive.setSpeed1(-cs.operator2.getLeftY(), -cs.operator2.getRightY());



  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    

  }
}
