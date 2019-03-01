
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

    isAutomated = true;

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

    //@TODO REmbermgsdf

    //driveControls();

    //armControls();

    //climbtakeControls();
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

    // System.out.println ("Camera ty: " + Camera.getEntry("ty").getDouble(0D));
    // System.out.println ("Camera tx: " + Camera.getEntry("tx").getDouble(0D));
    // System.out.println ("Camera ta: " + Camera.getEntry("ta").getDouble(0D));
    // System.out.println ("Camera tv: " + Camera.getEntry("tv").getDouble(0D));

    // System.out.println ("Camera Distance: " + Camera.getDistance() + " feet");

    /*
     * if (cs.driver.buttonPressed(Controller.DOWN_A)) { //drive.cameraDrive();
     * //drive.stopDrivePID(); } else
     */

    //driveControls();

    //armControls();

      // if (cs.operator1.buttonHeld(Controller.DOWN_A)) {
      //   arm.setExtenderTarget(3);
      //   arm.enableExtenderPID();
      // } else if (cs.operator1.buttonHeld(Controller.UP_Y)) {
      //   arm.setExtenderTarget(10);
      //   arm.enableExtenderPID();
      // } else {
      //   arm.disableExtenderPID();
      //   arm.setExtenderSpeed(cs.operator1.getLeftY() * .5);
      // }

      // arm.setElbowSpeed(cs.operator1.getRightY() * .25);

    //climbtakeControls();

   // automatedControls();

    if (cs.operator1.buttonHeld(Controller.SELECT)) {
      isAutomated = true;
    } else if (cs.operator1.buttonHeld(Controller.START)) {
      isAutomated = false;
    }

    if (isAutomated) {
      automatedControls();
      robotSafety();
      enablePids();
    } else {
      manualControls();
    }
    grabberControls();
    driveControls();

    

    

    // testControls();

    

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
      arm.setExtenderTarget(11.7);
      arm.autoSetWrist(WristMode.output);
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_B)) {
      // mid score
      arm.pidGoToAngle(15.3);
      arm.setExtenderTarget(0);
      arm.autoSetWrist(WristMode.output);
    } else if (cs.operator1.buttonHeld(Controller.DOWN_A)) {
      // low score
      arm.pidGoToAngle(-79.5);
      arm.setExtenderTarget(.7);
      arm.autoSetWrist(WristMode.output);

    } else if (cs.operator1.buttonHeld(Controller.LEFT_X)) {
      // intake - need to config
      //arm.pidGoToAngle(-79.5);
      //arm.setExtenderTarget(.7);
      arm.autoSetWrist(WristMode.intake);
    } else {
      


    }

    

    


    // climbtake

    if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {
      climb.setToPosition(IntakeClimber.VERTICAL_POSITION);
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT) {
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.RIGHT) {
      climb.setToPosition(IntakeClimber.IN_ANGLE);
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {
      climb.setToPosition(IntakeClimber.OUT_ANGLE);
    } else {
      
    }

  }

  private void manualControls () {

    if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {
      climb.setRotateSpeed(IntakeClimber.ROTATE_SPEED);
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {
      climb.setRotateSpeed(IntakeClimber.ROTATE_SPEED);
    } else {
      climb.setRotateSpeed(0);
    }

    if (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) {
      arm.setExtenderSpeed(ArmConstants.EXTENDER_SPEED);
    } else if (cs.operator1.buttonHeld(Controller.LEFT_TRIGGER)) {
      arm.setExtenderSpeed(-ArmConstants.EXTENDER_SPEED);
    } else {
      arm.setExtenderSpeed(0);
    }

    arm.setWristSpeed(cs.operator1.getLeftY());
    arm.setElbowSpeed(cs.operator1.getRightY());

  }

  private void grabberControls () {

    if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {
      arm.closeHatchGrabber();
      arm.setGrabberSparkSpeed(Grabber.INTAKE_OUTPUT_SPEED);
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {
      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(0);
    } else if (arm.grabberHasHatch()) {
      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(-Grabber.INTAKE_OUTPUT_SPEED);
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

        // ...or the arm wants to be in the shared space
      } else if (arm.getElbowTargetAngle() < ArmConstants.ELBOW_MIN_SAFE_ANGLE) {
        // tell the arm to go to a safe place instead. The climbtake may still move
        arm.setWristToArmAngle(WristMode.tucked);
        arm.setExtenderTarget(ArmConstants.MIN_EXTENDER_LENGTH);
        arm.setElbowTarget(ArmConstants.ELBOW_CLEARANCE_POSITION);
      }

      
    }

    if (Math.abs(arm.getElbowAngle() - arm.getElbowTargetAngle()) >= Arm.ELBOW_SAFE_ANGLE + 5.0) {
      arm.setWristToArmAngle(WristMode.tucked);
    }

    // Keep the extendo safe while in transit
    if ((Math.abs(arm.getElbowAngle() - arm.getElbowTargetAngle()) >= Arm.ELBOW_SAFE_ANGLE)) {
      arm.setExtenderTarget(ArmConstants.MIN_EXTENDER_LENGTH);
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

  private void armControls() {

    WristMode targetWristMode = WristMode.output;
    // if(cs.operator1.buttonHeld(Controller.UP_Y)) {
    // targetWristMode = WristMode.straight;
    // } else if (cs.operator1.buttonHeld(Controller.DOWN_A)) {
    // targetWristMode = WristMode.tucked;
    // }

    // if (targetWristMode == WristMode.nothing) {
    // arm.stopWristPID();
    // arm.setWristSpeed(cs.operator2.getLeftY() * 0.25);
    // } else {
    // arm.autoMoveWrist(targetWristMode);
    // }
    arm.autoSetWrist(WristMode.tucked);

    //

    if (cs.operator1.buttonHeld(Controller.DOWN_A)) {

      // pickup position
      arm.pidGoToAngle(ArmConstants.BALL_PICKUP_ANGLE);

      targetWristMode = WristMode.intake;

    } else if (cs.operator1.buttonHeld(Controller.RIGHT_B)) {

      // high ball
      arm.setArmToVerticalPosition(ArmConstants.BALL_HIGH);

    } else if (cs.operator1.buttonHeld(Controller.UP_Y)) {

      // mid ball
      arm.setArmToVerticalPosition(ArmConstants.BALL_MID);

    } else if (cs.operator1.buttonHeld(Controller.LEFT_X)) {

      // low ball
      arm.setArmToVerticalPosition(ArmConstants.BALL_LOW);

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {

      // cargo ball
      arm.setArmToVerticalPosition(ArmConstants.CARGO_BALL);

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.RIGHT) {

      // high hatchpanel
      arm.setArmToVerticalPosition(ArmConstants.HATCH_HIGH);

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {

      // mid hatchpanel
      arm.setArmToVerticalPosition(ArmConstants.HATCH_MID);

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT) {

      // low hatchpanel
      arm.setArmToVerticalPosition(ArmConstants.HATCH_LOW);

    } else {

      arm.disableElbowPID();
      arm.disableExtenderPID();

      // manual elbow and extender
      arm.setElbowSpeed(cs.operator1.getRightY() * 0.3);
      arm.setExtenderSpeed(cs.operator1.getLeftY() * 0.25);

      if (cs.operator1.buttonTapped(Controller.START)) {
        targetWristMode = WristMode.straight;
       
      } else {
        targetWristMode = WristMode.nothing;
      }

    }

    //arm.autoMoveWrist(targetWristMode);
    
    // if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {

    // arm.setGrabberSparkSpeed(-Grabber.INTAKE_OUTPUT_SPEED);

    // } else {
    // arm.setGrabberSparkSpeed(Grabber.INTAKE_OUTPUT_SPEED);
    // }

    // if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {

    // arm.retractHatchGrabber();

    // } else if (arm.getGrabberLimitSwitchLeft() &&
    // arm.getGrabberLimitSwitchRight()) {

    // arm.extendHatchGrabber();

    // }

  }

  // private void extenderCotroler() {


  //   if (cs.operator1.buttonHeld(Controller.UP_Y)) {
  //     arm.setArmDistance(ArmConstants.MAX_EXTENDER_LENGTH);
  //   } else if (cs.operator1.buttonHeld(Controller.LEFT_X)) {
  //     arm.setArmDistance(ArmConstants.MAX_EXTENDER_LENGTH/2);
  //   } else if (cs.operator1.buttonHeld(Controller.DOWN_A)) {
  //     arm.setArmDistance(0);
  //   }
  // }

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
    if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {
      // set climb to climb "position"
      climb.setToPosition(IntakeClimber.OUT_ANGLE);
      climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {
      // set climb to intake
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
    } else if (cs.operator1.buttonHeld(Controller.SELECT)) {
      // set climb to retract
      climb.setToPosition(IntakeClimber.IN_ANGLE);
      climb.goToSetPoint();
      climb.setFlywheel(0);
    } else {
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

    SmartDashboard.putNumber("Camera tangent distance", Camera.getDistance());
    SmartDashboard.putNumber("Climber Pot (RAW)", climb.anglePot.get());
    SmartDashboard.putNumber("Climber Pot (Angle)", climb.anglePot.getValue());
    SmartDashboard.putNumber("Elbow Pot (RAW)", arm.getRawElbow());
    SmartDashboard.putNumber("Elbow Pot (Angle)", arm.getElbowAngle());
    SmartDashboard.putNumber("Extender Pot (RAW)", arm.getRawExtender());
    SmartDashboard.putNumber("Extender Pot (Distance)", arm.getExtenderPosition());
    SmartDashboard.putNumber("Wrist Pot (RAW)", arm.getWirstPotRaw());
    SmartDashboard.putNumber("Wrist Pot (Angle)", arm.getWristAngle());
    SmartDashboard.updateValues();

  }

}
