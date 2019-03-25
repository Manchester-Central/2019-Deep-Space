
package frc.robot;

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
  PneumaticLift lift;

  boolean isAutomated;

  boolean[] galaxyBrain = { true, true, true, true, true };

  boolean elbowSetToPoint;
  boolean extenderSetToPoint;
  boolean wristSetToPoint;
  boolean climbSetToPoint;
  WristMode outputType;

  /**
   * 
   */
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

    //System.out.print("Enabled:" + pid.isEnabled() + "\t");
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

    Camera.changeCamMode(camState.driver);

    // if (cs.driver.buttonTapped(Controller.UP_Y)) {
    //   Camera.toggleCamState();

    // }

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
    
    automatedControls();
    grabberControls();
    driveControls();
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

    automatedControls();
    grabberControls();
    driveControls();
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
    if ( !cs.operator1.buttonHeld(Controller.RIGHT_BUMPER) &&  !cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {
      if (arm.grabberHasBall()) {
       outputType = WristMode.tilt;
     } else {
        outputType = WristMode.output;
      }
    }
  

    if (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) {

      if (outputType == WristMode.output) {
        outputType = WristMode.tilt;
      } else {
       outputType = WristMode.output;
      }

    }

    if (cs.operator1.buttonHeld(Controller.UP_Y)) {
      // high score

      arm.pidGoToAngle(72 + armModifier(Controller.UP_Y, outputType));
      arm.setArmPose(outputType, 13.7 + extenderModifier(Controller.UP_Y, outputType));
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_B)) {
      // mid score
      arm.pidGoToAngle(17.3 + armModifier(Controller.RIGHT_B, outputType));
      arm.setArmPose(outputType, 0 + extenderModifier(Controller.RIGHT_B, outputType));
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.START)) {
      // safe position above bottom
      arm.pidGoToAngle(-105.0);
      arm.setArmPose(WristMode.safe, 0);
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;  
    } else if (cs.operator1.buttonHeld(Controller.LEFT_TRIGGER)) {
      // cargo score
      arm.pidGoToAngle(0 + armModifier(Controller.LEFT_TRIGGER, WristMode.cargoShip));
      arm.setArmPose(WristMode.cargoShip, 0 + extenderModifier(Controller.LEFT_TRIGGER, WristMode.cargoShip));
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.DOWN_A)) {
      // low score
      arm.pidGoToAngle(-52.5 + armModifier(Controller.DOWN_A, outputType));
      arm.setArmPose(outputType, 4.1 + extenderModifier(Controller.DOWN_A, outputType));
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.buttonHeld(Controller.LEFT_X) || cs.operator1.getDPad() == DPadDirection.LEFT) {

      if (climb.getAngle() > IntakeClimber.INTAKE_ANGLE + 2.0) {
        arm.pidGoToAngle(-90.0);
        arm.setArmPose(WristMode.tucked, 0);
        //if (!climbSetToPoint) {
        //   climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
        //   climb.goToSetPoint();
        // }
      } else {// if (cs.operator1.buttonHeld(Controller.LEFT_BUMPER)) {
        arm.pidGoToAngle(-146.0);
        arm.setArmPose(WristMode.cargoIntake, 13.7);
       }// else {
      //   arm.pidGoToAngle(-138.0);
      //   arm.setArmPose(WristMode.intake, 11);
      // }

      // intake    
      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else if (cs.operator1.getDPad() == DPadDirection.DOWN && climb.getAngle() < IntakeClimber.INTAKE_ANGLE) {

      arm.pidGoToAngle(-146.0);
      arm.setArmPose(WristMode.cargoIntake, 13.7);

      elbowSetToPoint = true;
      extenderSetToPoint = true;
      wristSetToPoint = true;
    } else {
      
      elbowSetToPoint = false;
      extenderSetToPoint = false;
      wristSetToPoint = false;

    }
    enablePids();
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
        modifier = 10D;
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
        modifier = 0D;
        break;
      case Controller.UP_Y:
        modifier = -1.9D;
        break;
      case Controller.RIGHT_B:
        modifier = 0D;
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
      arm.closeHatchGrabber();
      arm.setGrabberSparkSpeed(-Grabber.INTAKE_OUTPUT_SPEED);
    } else if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {
      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(Grabber.INTAKE_OUTPUT_SPEED);
    } else if (arm.grabberHasHatch()) {
      arm.openHatchGrabber();
      arm.setGrabberSparkSpeed(0);
     // System.out.println("has hatch");
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

  /**
   * 
   * Controls for driving: Joysticks: driving (tank drive) X button: PID drive to
   * align w/ vision target
   * 
   */
  private void driveControls() {

    /**
     * 
     * Controls for driving: Joysticks: driving (tank drive) X button: PID drive to
     * align w/ vision target
     * 
     */

    double speedMultiplier = 0.5;

    if (cs.driver.buttonHeld(Controller.RIGHT_TRIGGER)) {
      speedMultiplier = 1;
    } else if (cs.driver.buttonHeld(Controller.RIGHT_BUMPER)) {
      speedMultiplier = .2;
    }

    // TODO test & configure 
    // if (cs.driver.buttonTapped(Controller.LEFT_X)) {
     // drive.resetCameraDrivePID();
    // }
    if (cs.driver.buttonHeld(Controller.LEFT_BUMPER)) {
      Camera.changePipeline(1);
      //drive.straightCameraDriveWithPID();
      drive.manualFollowCamera(cs.driver.getLeftY(), cs.driver.getRightY());
    } else if (cs.driver.getDPad() == DPadDirection.DOWN) {
      drive.stopDrivePID();
      drive.setSpeed(-0.5 * speedMultiplier, -0.4 * speedMultiplier);
      Camera.changePipeline(0);
    } else if (cs.driver.getDPad() == DPadDirection.UP) {
      drive.stopDrivePID();
      drive.setSpeed(0.5 * speedMultiplier, 0.4 * speedMultiplier);
      Camera.changePipeline(0);
    } else {
      drive.stopDrivePID();
      drive.setSpeed(cs.driver.getLeftY() * speedMultiplier, cs.driver.getRightY() * speedMultiplier);
      Camera.changePipeline(0);


    }

    //if (cs.operator1.buttonHeld(Controller.RIGHT_B)) {

      //Up to retract back in, Down to push up
      if (cs.driver.buttonHeld(Controller.UP_Y) ) {
      lift.setPositionIn();
      } else if (cs.driver.buttonHeld(Controller.DOWN_A)) {
      //if (isClimbingAllowed()) {
        lift.setPositionOut();
      //}
      }
    //}
  }

  //TODO look at before competition
  public boolean isClimbingAllowed () {
    return true;//(Timer.getMatchTime() <= 30);
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

  private void 
  climbtakeControls() {

    /**
     * 
     * Climb Controls: Left Trigger moves climb mech down Right Trigger moves climb
     * mech up
     */
    if (cs.operator1.buttonHeld(Controller.LEFT_X) && cs.operator1.getDPad() != DPadDirection.DOWN) {
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      climb.goToSetPoint();
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.DOWN) {
      // set climb to climb "position"
      climb.setToPosition(IntakeClimber.OUT_ANGLE);
      //climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
      climbSetToPoint = true;
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.UP) {
      climb.setToPosition(IntakeClimber.VERTICAL_POSITION);
      climb.goToSetPoint();
      climbSetToPoint = true;
    } else if (cs.operator1.getDPad() == Controller.DPadDirection.LEFT) {
      // set climb to intake
      climb.setToPosition(IntakeClimber.INTAKE_ANGLE);
      //climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      climb.goToSetPoint();
      climbSetToPoint = true;
    }  else {
      climb.setFlywheel(0);
      climb.stopPIDRotate();
      climbSetToPoint = false;
    }

    if (cs.operator1.buttonHeld(Controller.RIGHT_BUMPER)) {
      
      if (cs.operator1.getDPad() == DPadDirection.DOWN) {

        climb.setFlywheel(IntakeClimber.CLIMB_SPEED);
      } else {
        climb.setFlywheel(IntakeClimber.INTAKE_SPEED);
      }

    } else if (cs.operator1.buttonHeld(Controller.RIGHT_TRIGGER)) {
      climb.setFlywheel(-IntakeClimber.INTAKE_SPEED);
    } else if (climb.getAngle() < IntakeClimber.INTAKE_ANGLE - (IntakeClimber.INTAKE_ANGLE/2)) {
      climb.setFlywheel((cs.driver.getLeftY() + cs.driver.getRightY())/ 2) ;
    } else {
      climb.setFlywheel(0);
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
    SmartDashboard.putNumber("arm position: ", arm.getElbowAngle());
    SmartDashboard.putNumber("arm raw", arm.getRawElbow());
    //SmartDashboard.putNumber("Intake raw", climb.getRawRotateValue());

    //SmartDashboard.putNumber("Current Drive Target", drive.getSetPoint());

    SmartDashboard.putNumber("Current Encoder Inches Left", drive.getDistanceInchesL());
    // SmartDashboard.putNumber("Current Encoder Count Left",
    // drive.getDistanceTicksL());

    SmartDashboard.putNumber("Current Encoder Inches Right", drive.getDistanceInchesR());
    // SmartDashboard.putNumber("Current Encoder Count Right",
    // drive.getDistanceTicksR());

    // SmartDashboard.putNumber("Current p", drive.getP());
    // SmartDashboard.putNumber("Current i", drive.getI());
    // SmartDashboard.putNumber("Current d", drive.getD());
    // SmartDashboard.putNumber("Current f", drive.getF());

    // SmartDashboard.putString("Current Drive", drive.getDriveSpeeds());
    // SmartDashboard.putBoolean("L", arm.getGrabberLimitSwitchLeft());
    // SmartDashboard.putBoolean("R", arm.getGrabberLimitSwitchRight());

    

    SmartDashboard.updateValues();

  }

}
