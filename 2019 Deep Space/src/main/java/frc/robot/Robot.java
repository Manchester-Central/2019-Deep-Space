

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Camera;
import frc.Camera.camState;
/**
 * 
 */
public class Robot extends IterativeRobot {
  


  DriveBase drive;
  ControllerSecretary cs;
  Arm arm;
  IntakeClimber climb;
  Grabber grab;
  
  AnalogInput x;

  /**
   * 
   */
  @Override
  public void robotInit() {

    x = new AnalogInput(2);
    
    drive = new DriveBase();
    cs = new ControllerSecretary();
    SmartDashboard.putNumber("p-value", 0);
    SmartDashboard.putNumber("i-value", 0);
    SmartDashboard.putNumber("d-value", 0);
    SmartDashboard.putNumber("f-value", 0);
    SmartDashboard.putNumber("setpoint", 0);

    
  }

  /**
   * 
   */
  @Override
  public void robotPeriodic() {

    Camera.changeCamMode(camState.image);

    if (cs.driver.buttonPressed(Controller.UP_Y)) {
  Camera.toggleCamState();

    }

    //  System.out.println (Camera.GetHorizontalAngle()); 

    SmartDashboard.putNumber("Current Drive Target", drive.getSetPoint());

    SmartDashboard.putNumber("Current Encoder Inches Left", drive.getDistanceInchesL());
    SmartDashboard.putNumber("Current Encoder Count Left", drive.getDistanceTicksL());
    
    SmartDashboard.putNumber("Current Encoder Inches Right", drive.getDistanceInchesR());
    SmartDashboard.putNumber("Current Encoder Count Right", drive.getDistanceTicksR());

    SmartDashboard.updateValues();

    SmartDashboard.putNumber("Current p", drive.getP());
    SmartDashboard.putNumber("Current i", drive.getI());
    SmartDashboard.putNumber("Current d", drive.getD());
    SmartDashboard.putNumber("Current f", drive.getF());

    SmartDashboard.putString("Current Drive", drive.getDriveSpeeds());

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
    
    //drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5), SmartDashboard.getNumber("i-value", 0),
     //smartdashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value", 0));

    // drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));
     drive.setTolerance();
     drive.resetEncoders();
     drive.stopDrivePID();
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
    
    /*
    if (cs.driver.buttonPressed(Controller.DOWN_A)) {
      //drive.cameraDrive();
      //drive.stopDrivePID();
    } else 
    */

    driveControls();

    armControls();
    intakeControls();



    climbtakeControls();

    SmartDashboard.updateValues();
  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {



  }

  private void driveControls() {

    /**
     * 
     * Controls for driving:
     * Joysticks: driving (tank drive)
     * X button: PID drive to align w/ vision target
     * 
     */

    if (cs.driver.buttonPressed(Controller.LEFT_X)) {
      drive.startStraightCameraDriveWithPID();
    }

    if (cs.driver.buttonHeld(Controller.LEFT_X))
    {
     
     drive.straightCameraDriveWithPID();
     
   } else {
     drive.setSpeed(cs.driver.getLeftY(), cs.driver.getRightY());
     drive.stopDrivePID();
   }

  }

  private void armControls() { 

    if (cs.operator1.buttonHeld(Controller.DOWN_A)){

      //pickup ball

    } else if (cs.operator1.buttonHeld(Controller.RIGHT_B)){
  
      //high ball

    } else if (cs.operator1.buttonHeld(Controller.UP_Y)){
  
      //mid ball

    } else if (cs.operator1.buttonHeld(Controller.LEFT_X)){
  
      //low ball

    }  else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN){
  
      //pickup hatchpanel

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.RIGHT){
  
      //high hatchpanel

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.UP){
  
      //mid hatchpanel

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT){
  
      //low hatchpanel

    } else {

      //manual elbow and extender

    }
    
    // make grabber open 
    // make grabber output

  }

  private void climbtakeControls() { // does buttonPressed allow hold?



    /**
     * 
     * Climb Controls:
     * Left Trigger moves climb mech down
     * Right Trigger moves climb mech up
     */

    if (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) {
      
    } 
  }

  private void intakeControls() {

    // intake controls go here
    // left joystick
    // Declare grab.(insert method here)(parameter);

  }

  @Override
  public void disabledPeriodic() {

    //drive.resetEncoders();
    //\drive.stopDrivePID();

    SmartDashboard.putNumber("Camera tangent distance", Camera.getDistance());
    SmartDashboard.putNumber("analog out", x.getVoltage());
    
    //drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5), SmartDashboard.getNumber("i-value", 0),
     // SmartDashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value", 0));

    drive.setPIDValues(0.001, 0.00001, 0.000001, 0.0);

     drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));
  
     if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
       drive.resetEncoders();
     }

    SmartDashboard.updateValues();


  }


}
