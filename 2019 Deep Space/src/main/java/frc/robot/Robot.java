
package frc.robot;

import java.io.Console;
import java.text.DecimalFormat;

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
  Arm arm;
  IntakeClimber climb;

  boolean isAutomated;

  boolean[] galaxyBrain = { false, false, false, false, false };

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
    arm = new Arm();
    climb = new IntakeClimber();

    elbowSetToPoint = false;
    extenderSetToPoint = false;
    wristSetToPoint = false;
    climbSetToPoint = false;

    isAutomated = false;

    drive.setTolerance();

    drive.stopDrivePID();
    arm.disableElbowPID();
    arm.disableExtenderPID();
    climb.stopPIDRotate();
    arm.stopWristPID();

    SmartDashboard.putNumber("p-value", 0);
    SmartDashboard.putNumber("i-value", 0);
    SmartDashboard.putNumber("d-value", 0);
    SmartDashboard.putNumber("f-value", 0);
    SmartDashboard.putNumber("setpoint", 0);

  }

  public static void describePID(PIDController pid, String pidName, double input, double output) {

    DecimalFormat x = new DecimalFormat("#.0000");

    System.out.print(pidName + ":\t");
    System.out.print("P:" + pid.getP() + "\t");
    SmartDashboard.putNumber("P-" + pidName, pid.getP());
    System.out.print("I:" + pid.getI() + "\t");
    SmartDashboard.putNumber("I-" + pidName, pid.getI());
    System.out.print("D:" + pid.getD() + "\t");
    SmartDashboard.putNumber("D-" + pidName, pid.getD());
    System.out.print("F:" + pid.getF() + "\t");
    SmartDashboard.putNumber("F-" + pidName, pid.getF());
    System.out.print("input:" + x.format(input) + "\t");
    SmartDashboard.putNumber("Input-" + pidName, input);
    System.out.print("output:" + output + "\t");
    SmartDashboard.putNumber("Output-" + pidName, output);
    System.out.print("Target:" + pid.getSetpoint() + "\t");
    SmartDashboard.putNumber("Target-" + pidName, pid.getSetpoint());
    System.out.print("Error:" + pid.getError() + "\t");
    SmartDashboard.putNumber("Error-" + pidName, pid.getError());

    System.out.print("Enabled:" + pid.isEnabled() + "\t");
    SmartDashboard.putBoolean("Enabled-" + pidName, pid.isEnabled());
  }

  /**
   * 
   */
  @Override
  public void robotPeriodic() {

    if (cs.driver.buttonTapped(Controller.DOWN_A)) {
      galaxyBrain[0] = !galaxyBrain[0];
    } else if (cs.driver.buttonTapped(Controller.LEFT_X)) {
      galaxyBrain[1] = !galaxyBrain[1];
    } else if (cs.driver.buttonTapped(Controller.UP_Y)) {
      galaxyBrain[2] = !galaxyBrain[2];
    } else if (cs.driver.buttonTapped(Controller.RIGHT_B)) {
      galaxyBrain[3] = !galaxyBrain[3];
    } else if (cs.driver.buttonTapped(Controller.RIGHT_BUMPER)) {
      galaxyBrain[4] = !galaxyBrain[4];
    }

    if (galaxyBrain[0]) {
      arm.describeElbowPID();
    }
    if (galaxyBrain[1]) {
      arm.describeExtenderPID();
    }
    if (galaxyBrain[2]) {
      arm.describeWristPID();
    }
    if (galaxyBrain[3]) {
      climb.describeClimberPID();
    }
    if (galaxyBrain[4]) {
      drive.describeSelf();
    }

    System.out.println();

    Camera.changeCamMode(camState.image);

    if (cs.driver.buttonTapped(Controller.UP_Y)) {
      Camera.toggleCamState();

    }

    // System.out.println (Camera.GetHorizontalAngle());

    updateDashboard();

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

    if (cs.operator1.buttonHeld(Controller.SELECT)) {
      isAutomated = true;
    } else if (cs.operator1.buttonHeld(Controller.START)) {
      isAutomated = false;
    }
    
    // if (isAutomated) {
    //   automatedControls();
    //   robotSafety();
    //   enablePids();
    // } else {
      manualControls();
      //disablePids();
    // }
    grabberControls();
    driveControls();
  }

  @Override
  public void teleopInit() {

    // drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5),
    // SmartDashboard.getNumber("i-value", 0),
    // smartdashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value",
    // 0));

    // drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));

    drive.resetEncoders();
    drive.stopDrivePID();

    // arm.set

  }

  /**
   * 
   */
  @Override
  public void teleopPeriodic() {

   
    if (cs.operator1.buttonHeld(Controller.SELECT)) {
      isAutomated = true;
    } else if (cs.operator1.buttonHeld(Controller.START)) {
      isAutomated = false;
    }

    // if (isAutomated) {
    //   automatedControls();
    //   robotSafety();
    //   enablePids();
    // } else {
    //   manualControls();
       //disablePids();
    // }
    //manualControls();

    //TODO Connect autosetextender to automated controls
    automatedControls();
    grabberControls();
    //driveControls();
    climbtakeControls();
      
    

    
    

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  private void enablePids () {
    if (elbowSetToPoint) 
      arm.enableElbowPID();
    else 
      arm.disableElbowPID();

    if (extenderSetToPoint)
      arm.enableExtenderPID();
    else 
      arm.disableExtenderPID();

    if (wristSetToPoint)
      arm.enableWristPID();
    else 
      arm.stopWristPID();

    if (climbSetToPoint) 
      climb.goToSetPoint();
    else
      climb.stopPIDRotate();
   
   
  }

  private void disablePids () {
    arm.disableElbowPID();
    arm.disableExtenderPID();
    arm.stopWristPID();
    climb.stopPIDRotate();
  }

  private void automatedControls () {

    //Arm

    if (cs.operator1.buttonHeld(Controller.UP_Y)) {
      // high score
      arm.pidGoToAngle(70);
      arm.setArmPose(WristMode.output, 11.7);
      //arm.setExtenderTarget(11.7);
      //arm.autoSetWrist(WristMode.output);
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_B)) {
      // mid score
      arm.pidGoToAngle(15.3);
      arm.setArmPose(WristMode.output, 0);
//      arm.setExtenderTarget(0);
//     arm.autoSetWrist(WristMode.output);
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.DOWN_A)) {
      // low score
      arm.pidGoToAngle(-79.5);
//      arm.setExtenderTarget(.7);
      arm.setArmPose(WristMode.output, 0.7);
//      arm.autoSetWrist(WristMode.output);
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.LEFT_X)) {
      // intake - need to config
      //arm.pidGoToAngle(-79.5);
      //arm.setExtenderTarget(.7);
      // HATCH INTAKE arm.pidGoToAngle(-131.0);
      // HATCK INTAKE arm.setArmPose(WristMode.intake, 3);
      arm.pidGoToAngle(-150.0);
      arm.setArmPose(WristMode.intake, 10);    
//      arm.autoSetWrist(WristMode.intake);
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else {
      
      elbowSetToPoint = false;
      extenderSetToPoint = false;
      wristSetToPoint = false;

    }
    enablePids();

    // climbtake
/*
    if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {
      climb.setToPosition(IntakeClimber.VERTICAL_POSITION);
      climbSetToPoint = true;
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT) {
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climbSetToPoint = true;
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.RIGHT) {
      climb.setToPosition(IntakeClimber.IN_ANGLE);
      climbSetToPoint = true;
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {
      climb.setToPosition(IntakeClimber.OUT_ANGLE);
      climbSetToPoint = true;
    } else {
      climbSetToPoint = false;
    }
*/
  }

  private void manualControls () {

    // if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {
    //   climb.setRotateSpeed(IntakeClimber.ROTATE_SPEED);
    // } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {
    //   climb.setRotateSpeed(-IntakeClimber.ROTATE_SPEED);
    // } else {
    //   climb.setRotateSpeed(0);
    // }

    climbtakeControls();

    if (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) {
      arm.setExtenderSpeed(ArmConstants.EXTENDER_SPEED);
    } else if (cs.operator1.buttonHeld(Controller.LEFT_TRIGGER)) {
      arm.setExtenderSpeed(-ArmConstants.EXTENDER_SPEED);
    } else {
      arm.setExtenderSpeed(0);
    }

    arm.setWristSpeed(cs.operator1.getLeftY()*0.4);
    arm.setElbowSpeed(cs.operator1.getRightY());

  }

  private void grabberControls () {

    if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {
      // Intake 
      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(-Grabber.INTAKE_OUTPUT_SPEED);
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {
      arm.closeHatchGrabber();
      arm.setGrabberSparkSpeed(Grabber.INTAKE_OUTPUT_SPEED);
    } else if (!arm.grabberHasHatch()) {
      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(0);
      System.out.println("has hatch");
    } else {
      arm.setGrabberSparkSpeed(0);
    }

  }

  private void robotSafety() {

    if (climb.isClimbtakeUnsafe()) {

      if (arm.getElbowAngle() < ArmConstants.ELBOW_MIN_SAFE_ANGLE) {
        // put the arm in a safe place and dont allow the climbtake to move.
        arm.setWristToArmAngle(WristMode.tucked);
        arm.setExtenderTarget(ArmConstants.MIN_EXTENDER_LENGTH);
        arm.setElbowTarget(ArmConstants.ELBOW_CLEARANCE_POSITION);
        climb.setToPosition(climb.getAngle());
        elbowSetToPoint = true;
        extenderSetToPoint = true;
        wristSetToPoint = true;
        // ...or the arm wants to be in the shared space
      } else if (arm.getElbowTargetAngle() < ArmConstants.ELBOW_MIN_SAFE_ANGLE) {
        // tell the arm to go to a safe place instead. The climbtake may still move
        arm.setWristToArmAngle(WristMode.tucked);
        arm.setExtenderTarget(ArmConstants.MIN_EXTENDER_LENGTH);
        arm.setElbowTarget(ArmConstants.ELBOW_CLEARANCE_POSITION);
        elbowSetToPoint = true;
        extenderSetToPoint = true;
        wristSetToPoint = true;
      }

      
    }

    if (Math.abs(arm.getElbowAngle() - arm.getElbowTargetAngle()) >= Arm.ELBOW_SAFE_ANGLE + 5.0) {
      arm.setWristToArmAngle(WristMode.tucked);
      wristSetToPoint = true;
    }

    // Keep the extendo safe while in transit
    if ((Math.abs(arm.getElbowAngle() - arm.getElbowTargetAngle()) >= Arm.ELBOW_SAFE_ANGLE)) {
      arm.setExtenderTarget(ArmConstants.MIN_EXTENDER_LENGTH);
      extenderSetToPoint = true;
    }
  }

  private void driveControls() {

    /**
     * 
     * Controls for driving: Joysticks: driving (tank drive) X button: PID drive to
     * align w/ vision target
     * 
     */

    if (cs.driver.buttonTapped(Controller.LEFT_X)) {
      drive.resetCameraDrivePID();
    }

    if (cs.driver.buttonHeld(Controller.LEFT_X)) {

      drive.straightCameraDriveWithPID();

    } else if (cs.driver.buttonHeld(Controller.DOWN_A)) {

      drive.stopDrivePID();
      drive.setSpeed(-0.25, -0.25);

    } else {

      drive.stopDrivePID();
      drive.setSpeed(cs.driver.getLeftY(), cs.driver.getRightY());

    }

  }

 


  private void testControls() {

    // drive.setSpeed(cs.driver.getLeftY()* 0.25, cs.driver.getRightY()* 0.25);

    arm.setElbowSpeed(cs.operator1.getLeftY() * 0.25);
    arm.setExtenderSpeed(cs.operator1.getRightY() * 0.25);

    // arm.setWristSpeed(cs.operator2.getLeftY()* 0.25);
    // climb.setRotateSpeed(cs.operator2.getRightY()* 0.25);

    // if (cs.operator1.buttonTapped(Controller.UP_Y)) {
    // arm.extendHatchGrabber();
    // } else if (cs.operator1.buttonTapped(Controller.DOWN_A)) {
    // arm.retractHatchGrabber();
    // }

    // if (cs.operator2.getDPad().equals(DPadDirection.UP)) {
    // arm.setWristSpeed(0.1);
    // } else if (cs.operator2.getDPad().equals(DPadDirection.DOWN)) {
    // arm.setWristSpeed(-0.1);
    // }

  }

  private void climbtakeControls() {

    /**
     * 
     * Climb Controls: Left Trigger moves climb mech down Right Trigger moves climb
     * mech up
     */
    if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {
      // set climb to climb "position"
      climb.setToPosition(IntakeClimber.OUT_ANGLE);
      climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {
      climb.setToPosition(IntakeClimber.VERTICAL_POSITION);
      climb.goToSetPoint();
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.RIGHT) {
      climb.setToPosition(IntakeClimber.IN_ANGLE);
      climb.goToSetPoint();
    } 
    else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT) {
      // set climb to intake
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
    }  else {
      climb.setFlywheel(0);
      climb.stopPIDRotate();
    }
  }

  @Override
  public void disabledPeriodic() {

    drive.setPIDValues(0.001, 0.00001, 0.000001, 0.0);

    drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));

    if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      drive.resetEncoders();
    }

  }

  public void updateDashboard() {

    SmartDashboard.putBoolean("Beam Sensor", arm.grabberHasBall());
    SmartDashboard.putBoolean("Bump Sensor (Left)", arm.getGrabberLimitSwitchLeft());
    SmartDashboard.putBoolean("Bump Sensor (Right)", arm.getGrabberLimitSwitchRight());

    SmartDashboard.putNumber("Current Drive Target", drive.getSetPoint());

    SmartDashboard.putNumber("Current Encoder Inches Left", drive.getDistanceInchesL());
    // SmartDashboard.putNumber("Current Encoder Count Left",
    // drive.getDistanceTicksL());

    SmartDashboard.putNumber("Current Encoder Inches Right", drive.getDistanceInchesR());
    // SmartDashboard.putNumber("Current Encoder Count Right",
    // drive.getDistanceTicksR());

    SmartDashboard.putNumber("Current p", drive.getP());
    SmartDashboard.putNumber("Current i", drive.getI());
    SmartDashboard.putNumber("Current d", drive.getD());
    SmartDashboard.putNumber("Current f", drive.getF());

    SmartDashboard.putString("Current Drive", drive.getDriveSpeeds());

    SmartDashboard.updateValues();

  }

}
