

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Camera;

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

    //System.out.println ("Camera ty: " + Camera.getEntry("ty").getDouble(0D));
    //System.out.println ("Camera tx: " + Camera.getEntry("tx").getDouble(0D));
    //System.out.println ("Camera ta: " + Camera.getEntry("ta").getDouble(0D));
    //System.out.println ("Camera tv: " + Camera.getEntry("tv").getDouble(0D));

    System.out.println ("Camera Distance: " + Camera.getDistance() + " feet");

    drive.setSpeed(-cs.driver.getLeftY(), -cs.driver.getRightY());

    if (cs.driver.buttonPressed(Controller.DOWN_A)) {
      drive.cameraDrive();
    }
    

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    

  }
}
