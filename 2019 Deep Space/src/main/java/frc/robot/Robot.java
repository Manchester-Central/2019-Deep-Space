

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
    SmartDashboard.putNumber("p-value", 0);
    SmartDashboard.putNumber("i-value", 0);
    SmartDashboard.putNumber("d-value", 0);
    SmartDashboard.putNumber("setpoint", 0);

    
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


  @Override
  public void teleopInit() {
    
    drive.setPIDValues(SmartDashboard.getNumber("p-value", 0), SmartDashboard.getNumber("i-value", 0),
     SmartDashboard.getNumber("d-value", 0));

     drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 0));
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

   //System.out.println ("Camera Distance: " + Camera.getDistance() + " feet");

    

    if (cs.driver.buttonPressed(Controller.DOWN_A)) {
      drive.cameraDrive();
      drive.stopDrivePID();
    } else if (cs.driver.buttonPressed(Controller.LEFT_X)) {
      
      drive.drivePID();
      
    } else {
      drive.setSpeed(cs.driver.getLeftY(), cs.driver.getRightY());
      //drive.stopDrivePID();
    }

    drive.drivePID();
    SmartDashboard.updateValues();
    

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
    drive.stopDrivePID();
    
    drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5), SmartDashboard.getNumber("i-value", 0),
     SmartDashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value", 0));

     drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));
  
    SmartDashboard.updateValues();


  }


}
