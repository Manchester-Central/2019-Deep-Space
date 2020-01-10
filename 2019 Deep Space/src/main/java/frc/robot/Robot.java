
package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.chaos.Camera;
import frc.chaos.Camera.camState;
import frc.robot.Arm.WristMode;
import frc.robot.Controller.DPadDirection;

public class Robot extends IterativeRobot {

  DriveBase drive;
  ControllerSecretary cs;
  Arm arm;
  IntakeClimber climb;
  PneumaticLift lift;

  boolean isAutomated;

  boolean elbowSetToPoint;
  boolean extenderSetToPoint;
  boolean wristSetToPoint;
  boolean climbSetToPoint;
  WristMode outputType;

  ///
  ///
  // Iterative Robot template functions:
  ///
  ///

  @Override
  public void robotInit() {
    outputType = WristMode.output;

    drive = new DriveBase();
    cs = new ControllerSecretary();
    arm = new Arm();
    climb = new IntakeClimber();
    lift = new PneumaticLift();

    elbowSetToPoint = false;
    extenderSetToPoint = false;
    wristSetToPoint = false;
    climbSetToPoint = false;

    drive.setTolerance();

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

  @Override
  public void autonomousInit() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void robotPeriodic() {

    Camera.changeCamMode(camState.image);
    SmartDashboard.putBoolean("Within Hatch Scoring Distance", drive.withinScoringDistance());

    updateDashboard();

  }

  @Override
  public void autonomousPeriodic() {


    robotControl();

  }

  @Override
  public void teleopPeriodic() {

    robotControl();

  }

  @Override
  public void disabledInit() {
    arm.disableElbowPID();
    arm.disableExtenderPID();
    arm.stopWristPID();
    climb.stopPIDRotate();
  }

  @Override
  public void disabledPeriodic() {

    if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      drive.resetEncoders();
    }

  }

  ///
  // Major system functions:
  ///

  private void robotControl () {
    armControls();
    grabberControls();
    driveControls();
    climbtakeControls();
  }

  private void armControls () {

    // reacts to the prescence of a ball unless inputing/outputing
    if ( !cs.operator1.buttonHeld(Controller.RIGHT_BUMPER) &&
         !cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {

      if (arm.grabberHasBall()) {
       outputType = WristMode.tilt;
     } else {
        outputType = WristMode.output;
      }
    }

    // inverses current wrist output type
    if (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) {
      outputType = outputType.flip(WristMode.tilt, WristMode.output);
    }

    // arm position based on user control
    armEnumeration();

  }

  private void grabberControls () {
    if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) { // output

      arm.closeHatchGrabber();
      arm.setGrabberSparkSpeed(-Grabber.INTAKE_OUTPUT_SPEED);

    } else if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) { // intake

      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(Grabber.INTAKE_OUTPUT_SPEED);

    } else if (arm.grabberHasHatch()) { // auto hatch grab

      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(0);

    } else {

      arm.setGrabberSparkSpeed(0);

    }
  }

  private void driveControls() {

    // speed modes: fast, 1; normal, .5; slow, .2
    double speedMultiplier = 0.5;
    if (cs.driver.buttonHeld(Controller.RIGHT_TRIGGER)) {
      speedMultiplier = 1;
    } else if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      speedMultiplier = .2;
    }

    // drive modes
    if (cs.driver.buttonHeld(Controller.LEFT_BUMPER)) { // camera assisted manual drive

      Camera.changePipeline(Camera.CAMERA_VISION);
      drive.manualFollowCamera(cs.driver.getLeftY() * speedMultiplier, cs.driver.getRightY() * speedMultiplier);

    } else {

      Camera.changePipeline(Camera.DRIVER_VISION);

      if (cs.driver.getDPad() == DPadDirection.DOWN) { // straight backwards

        drive.setSpeed(-0.5 * speedMultiplier, -0.4 * speedMultiplier);

      } else if (cs.driver.getDPad() == DPadDirection.UP) { // straight forwards

        drive.setSpeed(0.5 * speedMultiplier, 0.4 * speedMultiplier);

      } else { // normal manual drive

        drive.setSpeed(cs.driver.getLeftY() * speedMultiplier, cs.driver.getRightY() * speedMultiplier);

      }
  }

    //Up to retract back in, Down to push up
    if (cs.driver.buttonHeld(Controller.UP_Y) ) {
      lift.setPositionIn();
    } else if (cs.driver.buttonHeld(Controller.DOWN_A)) {
      lift.setPositionOut();
    }
  }

  private void climbtakeControls() {

    // climb positions
    climbSetToPoint = true;
    if (cs.operator1.buttonHeld(Controller.LEFT_X)
     && cs.operator1.getDPad() != DPadDirection.DOWN) { // climb 2 position

      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climb.goToSetPoint();

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) { // climb 1 position

      climb.setToPosition(IntakeClimber.OUT_ANGLE);
      climb.goToSetPoint();

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.UP
                || cs.operator1.buttonHeld(Controller.DOWN_A)
                || cs.operator1.buttonHeld(Controller.RIGHT_B)
                || cs.operator1.buttonHeld(Controller.UP_Y)) { // vertical position

      climb.setToPosition(IntakeClimber.VERTICAL_POSITION);
      climb.goToSetPoint();

    } else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT) { // intake position

      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climb.goToSetPoint();

    } else { // climb stop

      climb.setFlywheel(0);
      climb.stopPIDRotate();
      climbSetToPoint = false;

    }

    // Flywheel speeds
    if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) { // Intake & climb

      if (cs.operator1.getDPad() == DPadDirection.DOWN) {
        climb.setFlywheel(IntakeClimber.CLIMB_SPEED);
      } else {
        climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      }

    } else if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) { // Output

      climb.setFlywheel(-IntakeClimber.INTAKE_SPEED);

    } else if (climb.getAngle() <
              IntakeClimber.INTAKE_ANGLE - (IntakeClimber.INTAKE_ANGLE/2)
              || cs.operator1.getDPad() == DPadDirection.DOWN) { // Driver climb override

      climb.setFlywheel((cs.driver.getLeftY() + cs.driver.getRightY())/ 2) ;

    } else { // Stop

      climb.setFlywheel(0);

    }
  }


  ///
  ///
  // Supplementary functions:
  ///
  ///


  private void armEnumeration () {

    elbowSetToPoint = true;
    extenderSetToPoint = true;
    wristSetToPoint = true;

    if (cs.operator1.buttonHeld(Controller.UP_Y)) { // high score


      arm.pidGoToAngle(72 + armModifier(Controller.UP_Y, outputType));
      arm.setArmPose(outputType, 13.7 + extenderModifier(Controller.UP_Y, outputType));
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_B)) { // mid score

      arm.pidGoToAngle(17.3 + armModifier(Controller.RIGHT_B, outputType));
      arm.setArmPose(outputType, 0 + extenderModifier(Controller.RIGHT_B, outputType));

    } else if (cs.operator1.buttonHeld(Controller.START)) { // safe position above bottom

      arm.pidGoToAngle(-103.0);
      arm.setArmPose(WristMode.safe, 0);

    } else if (cs.operator1.buttonHeld(Controller.LEFT_TRIGGER)) { // cargo score

      WristMode outputTypeCargo =
       (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) ? WristMode.tilt : WristMode.output;

      arm.pidGoToAngle(0 + armModifier(Controller.LEFT_TRIGGER, outputTypeCargo));
      arm.setArmPose(WristMode.cargoShip, 0 + extenderModifier(Controller.LEFT_TRIGGER, outputTypeCargo));

    } else if (cs.operator1.buttonHeld(Controller.DOWN_A)) { // low score/hatch intake


      arm.pidGoToAngle(-52.5 + armModifier(Controller.DOWN_A, outputType));
      arm.setArmPose(outputType, 4.1 + extenderModifier(Controller.DOWN_A, outputType));

    } else if (cs.operator1.buttonHeld(Controller.LEFT_X)
     || cs.operator1.getDPad() == DPadDirection.LEFT) { // ball intake

      if (climb.getAngle() > IntakeClimber.INTAKE_ANGLE + 2.0) {
        arm.pidGoToAngle(-90.0);
        arm.setArmPose(WristMode.tucked, 0);
      } else {
        arm.pidGoToAngle(-146.0);
        arm.setArmPose(WristMode.cargoIntake, 13.7);
       }

    } else if (cs.operator1.getDPad() == DPadDirection.DOWN
    && climb.getAngle() < IntakeClimber.INTAKE_ANGLE) { // climb position

      arm.pidGoToAngle(-146.0);
      arm.setArmPose(WristMode.cargoIntake, 13.7);

    } else {

      elbowSetToPoint = false;
      extenderSetToPoint = false;
      wristSetToPoint = false;
    }
    enablePids();
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

  public double armModifier (int button, WristMode mode) {
    if (mode == WristMode.output)
      return 0D;
    double modifier = 0D;
    switch (button) {
      case Controller.DOWN_A:
        modifier = 31D - 24D;
        break;
      case Controller.UP_Y:
        modifier = 8D;
        break;
      case Controller.RIGHT_B:
        modifier = 15D;
        break;
      case Controller.LEFT_TRIGGER:
        modifier = 0D;
        break;
      default:
        modifier = 0D;
        break;
    }
    return modifier;
  }

  public double extenderModifier (int button, WristMode mode) {
    if (mode == WristMode.output)
      return 0D;
    double modifier = 0D;
    switch (button) {
      case Controller.DOWN_A:
        modifier = -4.0D;
        break;
      case Controller.UP_Y:
        modifier = -1.9D;
        break;
      case Controller.RIGHT_B:
        modifier = 0D;
        break;
      case Controller.LEFT_TRIGGER:
        modifier = 2D;
        break;
      default:
        modifier = 0D;
        break;
    }
    return modifier;
  }


  ///
  ///
  // Debug and informational functions:
  ///
  ///


  public static void describePID(PIDController pid, String pidName, double input, double output) {

    SmartDashboard.putNumber("Input-" + pidName, input);
    SmartDashboard.putBoolean("Enabled-" + pidName, pid.isEnabled());
  }

  public void updateDashboard() {

    SmartDashboard.putBoolean("Beam Sensor", arm.grabberHasBall());
    SmartDashboard.putBoolean("Bump Sensor (Left)", arm.getGrabberLimitSwitchLeft());
    SmartDashboard.putBoolean("Bump Sensor (Right)", arm.getGrabberLimitSwitchRight());
    SmartDashboard.putNumber("arm position: ", arm.getElbowAngle());
    SmartDashboard.putNumber("arm raw", arm.getRawElbow());

    SmartDashboard.putNumber("Current Encoder Inches Left", drive.getDistanceInchesL());

    SmartDashboard.putNumber("Current Encoder Inches Right", drive.getDistanceInchesR());

    SmartDashboard.putNumber("climb raw", climb.getRawRotateValue());
    SmartDashboard.putNumber("climb", climb.getAngle());

    SmartDashboard.putNumber("pipeline", Camera.getEntry("getpipe").getDouble(131));

    SmartDashboard.updateValues();

  }


  ///
  // Unused and deprecated functions:
  ///


  private void manualControls () {

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

  public boolean isClimbingAllowed () {
    return true;//(Timer.getMatchTime() <= 30);
  }

  private void testControls() {

    drive.setSpeed(cs.driver.getLeftY()* 0.25, cs.driver.getRightY()* 0.25);

    arm.setElbowSpeed(cs.operator1.getLeftY() * 0.25);
    arm.setExtenderSpeed(cs.operator1.getRightY() * 0.25);

    arm.setWristSpeed(cs.operator2.getLeftY()* 0.25);
    climb.setRotateSpeed(cs.operator2.getRightY()* 0.25);

    if (cs.operator1.buttonTapped(Controller.UP_Y)) {
    arm.openHatchGrabber();
    } else if (cs.operator1.buttonTapped(Controller.DOWN_A)) {
    arm.closeHatchGrabber();
    }

    if (cs.operator2.getDPad().equals(DPadDirection.UP)) {
    arm.setWristSpeed(0.1);
    } else if (cs.operator2.getDPad().equals(DPadDirection.DOWN)) {
    arm.setWristSpeed(-0.1);
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

  private void disablePids () {
    arm.disableElbowPID();
    arm.disableExtenderPID();
    arm.stopWristPID();
    climb.stopPIDRotate();
  }

}
