
package frc.robot;

import java.io.Console;
import java.text.DecimalFormat;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Camera;
import frc.Camera.camState;
import frc.robot.Arm.WristMode;
import frc.robot.Controller.DPadDirection;

/**
 * 
 */
public class Robot extends IterativeRobot {

  DriveBase drive;
  ControllerSecretary cs;
  // Arm arm;
  // IntakeClimber climb;

  boolean isAutomated;

  boolean[] galaxyBrain = { true, true, true, true, true };

  boolean elbowSetToPoint;
  boolean extenderSetToPoint;
  boolean wristSetToPoint;
  boolean climbSetToPoint;

  /**
   * 
   */
  @Override
  public void robotInit() {

    drive = new DriveBase();
    cs = new ControllerSecretary();
    // arm = new Arm();
    // climb = new IntakeClimber();

    elbowSetToPoint = false;
    extenderSetToPoint = false;
    wristSetToPoint = false;
    climbSetToPoint = false;

    isAutomated = false;

    drive.setTolerance();

    drive.stopDrivePID();

    SmartDashboard.putNumber("p-value", 0);
    SmartDashboard.putNumber("i-value", 0);
    SmartDashboard.putNumber("d-value", 0);
    SmartDashboard.putNumber("f-value", 0);
    SmartDashboard.putNumber("setpoint", 0);
    //Camera.switchPipelines(1);

  }

  public static void describePID(PIDController pid, String pidName, double input, double output) {

    DecimalFormat x = new DecimalFormat("#.0000");

    // System.out.print(pidName + ":\t");
    // System.out.print("P:" + pid.getP() + "\t");
    // SmartDashboard.putNumber("P-" + pidName, pid.getP());
    // System.out.print("I:" + pid.getI() + "\t");
    // SmartDashboard.putNumber("I-" + pidName, pid.getI());
    // System.out.print("D:" + pid.getD() + "\t");
    // SmartDashboard.putNumber("D-" + pidName, pid.getD());
    // System.out.print("F:" + pid.getF() + "\t");
    // SmartDashboard.putNumber("F-" + pidName, pid.getF());
    //System.out.print("input:" + x.format(input) + "\t");
    SmartDashboard.putNumber("Input-" + pidName, input);
    // System.out.print("output:" + output + "\t");
    // SmartDashboard.putNumber("Output-" + pidName, output);
    // System.out.print("Target:" + pid.getSetpoint() + "\t");
    // SmartDashboard.putNumber("Target-" + pidName, pid.getSetpoint());
    // System.out.print("Error:" + pid.getError() + "\t");
    // SmartDashboard.putNumber("Error-" + pidName, pid.getError());

    System.out.print("Enabled:" + pid.isEnabled() + "\t");
    SmartDashboard.putBoolean("Enabled-" + pidName, pid.isEnabled());
   
  }

  /**
   * 
   */
  @Override
  public void robotPeriodic() {
   // Camera.switchPipelines(1);
    //System.out.println(Camera.GetHorizontalAngle());
   // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    // if (cs.driver.buttonTapped(Controller.DOWN_A)) {
    //   galaxyBrain[0] = !galaxyBrain[0];
    // } else if (cs.driver.buttonTapped(Controller.LEFT_X)) {
    //   galaxyBrain[1] = !galaxyBrain[1];
    // } else if (cs.driver.buttonTapped(Controller.UP_Y)) {
    //   galaxyBrain[2] = !galaxyBrain[2];
    // } else if (cs.driver.buttonTapped(Controller.RIGHT_B)) {
    //   galaxyBrain[3] = !galaxyBrain[3];
    // } else if (cs.driver.buttonTapped(Controller.RIGHT_BUMPER)) {
    //   galaxyBrain[4] = !galaxyBrain[4];
    // } 

    SmartDashboard.putBoolean("In Hatch Range", drive.withinHatchRange());

    // if (galaxyBrain[0]) {
    //   arm.describeElbowPID();
    // }
    // if (galaxyBrain[1]) {
    //   arm.describeExtenderPID();
    // }
    // if (galaxyBrain[2]) {
    //   arm.describeWristPID();
    // }
    // if (galaxyBrain[3]) {
    //   climb.describeClimberPID();
    // }
    // if (galaxyBrain[4]) {
    //   drive.describeSelf();
    // }

    ///Camera.changeCamMode(camState.driver);

    // if (cs.driver.buttonTapped(Controller.UP_Y)) {
    //   Camera.toggleCamState();

    // }

    System.out.println(Camera.getDistance());

    if (cs.driver.buttonHeld(Controller.LEFT_BUMPER) || cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      //drive.resetCameraDrivePID();
      Camera.switchPipelines(Camera.CAMERA_VISION);
    } else {
      Camera.switchPipelines(Camera.DRIVER_VISION);
    }
    
    SmartDashboard.updateValues();
    

  }

  /**
   * 
   */
  @Override
  public void autonomousInit() {

    drive.resetEncoders();
    drive.stopDrivePID();
  }

  /**
   * 
   */
  @Override
  public void autonomousPeriodic() {

    // if (cs.operator1.buttonHeld(Controller.SELECT)) {
    //   isAutomated = true;
    // } else if (cs.operator1.buttonHeld(Controller.START)) {
    //   isAutomated = false;
    // }
    
  }

  @Override
  public void teleopInit() {

    // drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5),
    // SmartDashboard.getNumber("i-value", 0),
    // smartdashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value",
    // 0));

    // drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));

    //drive.resetEncoders();
    //drive.stopDrivePID();

    // arm.set

  }

  /**
   * 
   */
  @Override
  public void teleopPeriodic() {

   // testControls();
    //driveControls();

    //drive.stopDrivePID();


    // System.out.println(Camera.getDistance());

    // if (cs.driver.buttonHeld(Controller.LEFT_BUMPER) || cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
    //   //drive.resetCameraDrivePID();
    //   Camera.switchPipelines(Camera.CAMERA_VISION);
    //  // drive.manualFollowCamera(cs.driver.getRightY(), cs.driver.getLeftY());
    //   //System.out.println (Camera.getDistanceFromArea());
      
    // } else {
    //   Camera.switchPipelines(Camera.DRIVER_VISION);
    //  // drive.setSpeed(cs.driver.getRightY(), cs.driver.getLeftY());
    // }
    

  }

  private void driveControls() {

    /**
     * 
     * Controls for driving: Joysticks: driving (tank drive) X button: PID drive to
     * align w/ vision target
     * 
     */

    if (cs.driver.buttonTapped(Controller.LEFT_X)) {
      //drive.resetCameraDrivePID();
      Camera.switchPipelines(3);
    }

    if (cs.driver.buttonHeld(Controller.LEFT_X)) {

     // drive.straightCameraDriveWithPID();
      drive.cameraDrive();
    } else {

      drive.stopDrivePID();
      Camera.switchPipelines(2);

      if (cs.driver.buttonHeld(Controller.LEFT_BUMPER)) {
        drive.setSpeed(cs.driver.getLeftY(), cs.driver.getRightY()); 
      } else if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
        drive.setSpeed(cs.driver.getLeftY() * .2, cs.driver.getRightY() * .2); 
      } else {
        drive.setSpeed(cs.driver.getLeftY() * .5, cs.driver.getRightY() * .5);
      }


    }

  }

 


  private void testControls() {

    // drive.setSpeed(cs.driver.getLeftY()* 0.25, cs.driver.getRightY()* 0.25);

    if ( cs.driver.buttonHeld(Controller.DOWN_A)) {
      Camera.switchPipelines(0);
    } else if (cs.driver.buttonHeld(Controller.RIGHT_B)) {
      Camera.switchPipelines(1);
    }

  }

  @Override
  public void disabledPeriodic() {

    // drive.setPIDValues(0.001, 0.00001, 0.000001, 0.0);

    // drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));

    // if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
    //   drive.resetEncoders();
    // }

  }


}
