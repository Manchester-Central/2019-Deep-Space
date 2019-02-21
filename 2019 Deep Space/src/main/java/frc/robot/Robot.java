
package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
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

  /**
   * 
   */
  @Override
  public void robotInit() {

    drive = new DriveBase();
    cs = new ControllerSecretary();
    arm = new Arm();
    climb = new IntakeClimber();

    drive.setTolerance();

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

    driveControls();

    armControls();

    climbtakeControls();
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

    //arm.set

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

    // driveControls();

    // armControls();

    // climbtakeControls();

    testControls();
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
      arm.setElbowSpeed(cs.operator1.getRightY());
      arm.setExtenderSpeed(cs.operator1.getLeftY());

      if (cs.operator1.buttonPressed(Controller.START)) {
        targetWristMode = WristMode.straight;
       
      } else {
        targetWristMode = WristMode.tucked;
      }

      

    }

    arm.autoMoveWrist(targetWristMode);
    
    if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {

      arm.setGrabberSparkSpeed(Grabber.INTAKE_OUTPUT_SPEED);

    } else {
      arm.setGrabberSparkSpeed(-Grabber.INTAKE_OUTPUT_SPEED);
    }

    if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {

      arm.retractHatchGrabber();

    } else if (arm.getGrabberLimitSwitchLeft() && arm.getGrabberLimitSwitchRight()) {

      arm.extendHatchGrabber();

    }



  }

  private void testControls() {


    drive.setSpeed(cs.driver.getLeftY()* 0.25, cs.driver.getRightY()* 0.25);

    arm.setElbowSpeed(cs.operator1.getLeftY() * 0.25);
    arm.setExtenderSpeed(cs.operator1.getRightY()* 0.25);

    arm.pidGoToAngle(0.0);
    
    arm.setWristSpeed(cs.operator2.getLeftY()* 0.25);
    climb.setRotateSpeed(cs.operator2.getRightY()* 0.25);

    if (cs.operator1.buttonPressed(Controller.UP_Y)) {
      arm.extendHatchGrabber();
    } else if (cs.operator1.buttonPressed(Controller.DOWN_A)) {
      arm.retractHatchGrabber();
    }

    // if (cs.operator2.getDPad().equals(DPadDirection.UP)) {
    //   arm.setWristSpeed(0.1);
    // } else if (cs.operator2.getDPad().equals(DPadDirection.DOWN)) {
    //   arm.setWristSpeed(-0.1);
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
      climb.stopPIDRotate();
      climb.setRotateSpeed(IntakeClimber.ROTATE_SPEED);
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

    drive.setPIDValues(0.001, 0.00001, 0.000001, 0.0);

    drive.setDriveDistance(SmartDashboard.getNumber("setpoint", 12.0));

    if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      drive.resetEncoders();
    }

  }

  public void updateDashboard() {

    SmartDashboard.putBoolean("Beam Sensor", arm.getGrabberBeamSensor());
    SmartDashboard.putBoolean("Bump Sensor (Left)", arm.getGrabberLimitSwitchLeft());
    SmartDashboard.putBoolean("Bump Sensor (Right)", arm.getGrabberLimitSwitchRight());

    SmartDashboard.putNumber("Current Drive Target", drive.getSetPoint());

    SmartDashboard.putNumber("Current Encoder Inches Left", drive.getDistanceInchesL());
    //SmartDashboard.putNumber("Current Encoder Count Left", drive.getDistanceTicksL());

    SmartDashboard.putNumber("Current Encoder Inches Right", drive.getDistanceInchesR());
   // SmartDashboard.putNumber("Current Encoder Count Right", drive.getDistanceTicksR());


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
