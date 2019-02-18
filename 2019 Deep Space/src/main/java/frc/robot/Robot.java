
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
  Wrist wrist;
  Grabber grab;
  boolean armSet;

  /**
   * 
   */
  @Override
  public void robotInit() {

    armSet = true;

    drive = new DriveBase();
    cs = new ControllerSecretary();
    arm = new Arm();
    climb = new IntakeClimber();
    wrist = new Wrist();
    grab = new Grabber();
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

    // System.out.println (Camera.GetHorizontalAngle());

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

    // drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5),
    // SmartDashboard.getNumber("i-value", 0),
    // smartdashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value",
    // 0));

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

    // System.out.println ("Camera ty: " + Camera.getEntry("ty").getDouble(0D));
    // System.out.println ("Camera tx: " + Camera.getEntry("tx").getDouble(0D));
    // System.out.println ("Camera ta: " + Camera.getEntry("ta").getDouble(0D));
    // System.out.println ("Camera tv: " + Camera.getEntry("tv").getDouble(0D));

    // System.out.println ("Camera Distance: " + Camera.getDistance() + " feet");

    /*
     * if (cs.driver.buttonPressed(Controller.DOWN_A)) { //drive.cameraDrive();
     * //drive.stopDrivePID(); } else
     */

    driveControls();

    armControls();

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
     * Controls for driving: Joysticks: driving (tank drive) X button: PID drive to
     * align w/ vision target
     * 
     */

    if (cs.driver.buttonPressed(Controller.LEFT_X)) {
      drive.startStraightCameraDriveWithPID();
    }

    if (cs.driver.buttonHeld(Controller.LEFT_X)) {

      drive.straightCameraDriveWithPID();

    } else {
      drive.setSpeed(cs.driver.getLeftY(), cs.driver.getRightY());
      drive.stopDrivePID();
    }

  }

  private void armControls() {

    armSet = true;

    if (cs.operator1.buttonHeld(Controller.DOWN_A)) {

      // pickup ball
      arm.pidGoToAngle(ArmConstants.BALL_PICKUP_ANGLE);
      arm.setArmDistance(0);
      arm.enableExtenderPID();

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
      arm.setElbowSpeed(cs.operator1.getRightY());
      arm.setElbowSpeed(cs.operator1.getLeftY());
      armSet = false;

      if (cs.operator1.buttonPressed(Controller.START)) {
        wrist.setSetPoint(Math.toRadians(Wrist.DEFAULT_ANGLE));
        wrist.goToSetPoint();
      } else {
        wrist.stopWristPID();
      }

    }

    if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {

      grab.setSpark(Grabber.INTAKE_OUTPUT_SPEED);

    } else {
      grab.setSpark(-Grabber.INTAKE_OUTPUT_SPEED);
    }

    if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {

      grab.retractHatchGrabber();

    } else if (grab.getLimitSwitchLeft() && grab.getLimitSwitchRight()) {

      grab.extendHatchGrabber();

    }

  }

  private void climbtakeControls() { // does buttonPressed allow hold?

    /**
     * 
     * Climb Controls: Left Trigger moves climb mech down Right Trigger moves climb
     * mech up
     */
    if (armSet) {
      climb.setToPosition(IntakeClimber.OUT_ANGLE);
      climb.setFlywheel(0);
      climb.goToSetPoint();
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {
      // set climb to climb "position"
      climb.stopPIDRotate();
      climb.setIntake(IntakeClimber.ROTATE_SPEED);
      climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {
      // set climb to intake
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
    } else if (cs.operator1.buttonHeld(Controller.SELECT)) {
      // set climb to retract
      climb.setToPosition(IntakeClimber.DOWN_ANGLE);
      climb.goToSetPoint();
      climb.setFlywheel(0);
    } else {
      climb.setFlywheel(0);
      climb.stopPIDRotate();
    }
  }

  @Override
  public void disabledPeriodic() {

    // drive.resetEncoders();
    // \drive.stopDrivePID();

    SmartDashboard.putNumber("Camera tangent distance", Camera.getDistance());

    // drive.setPIDValues(SmartDashboard.getNumber("p-value", 0.5),
    // SmartDashboard.getNumber("i-value", 0),
    // SmartDashboard.getNumber("d-value", 0), SmartDashboard.getNumber("f-value",
    // 0));

    drive.setPIDValues(0.001, 0.00001, 0.000001, 0.0);

    drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));

    if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      drive.resetEncoders();
    }

    SmartDashboard.updateValues();

  }

}
