package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.commands.SystemCheck;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class RobotContainer {
  
  // ==========================================
  // 1. SUBSYSTEM INSTANCES
  // ==========================================
  // We instantiate all physical robot mechanisms here as static final objects.
  public static final Drive drive = new Drive();
  public static final Intake intake = new Intake();
  public static final Agitator agitator = new Agitator();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  public static final Climber climber = new Climber();
  public static final LED led = new LED(indexer, intake, shooter); 
  public static final Limelight limelight = new Limelight();

  // Driver Input Controller (Port 0)
  private final Joystick driverController = new Joystick(0);
  private Boolean cachedXboxMode = null;
  public static boolean isDemoMode = false;

  // SmartDashboard Choosers
  private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
  private final SendableChooser<String> controllerChooser = new SendableChooser<>();

  public RobotContainer() {
    // Controller Auto-Detection Setup
    controllerChooser.setDefaultOption("Auto-Detect Controller", "AUTO");
    controllerChooser.addOption("Force Xbox Mode", "XBOX");
    controllerChooser.addOption("Force GameCube Mode", "GAMECUBE");
    SmartDashboard.putData("Controller Type", controllerChooser);

    // Starts the capture and sends the stream directly to Shuffleboard
    UsbCamera driverCam = CameraServer.startAutomaticCapture();
    // CRITICAL: Clamp the resolution and framerate so the RoboRIO doesn't crash!
    driverCam.setResolution(/*160, 120*/320,240); 
    driverCam.setFPS(15);
      
    // Drive Mode Selection
    driveModeChooser.setDefaultOption("Racing Drive (Triggers + Aim-Bot)", "RACING");
    driveModeChooser.addOption("Tank Drive (Left Y, Right Y)", "TANK");
    SmartDashboard.putData("Drive Mode", driveModeChooser);
    SmartDashboard.putNumber("Tuning/Low Shoot Speed", 0.38);
    // Shuffleboard System Check Button
    SmartDashboard.putData("Run Mantis System Check", 
        new SystemCheck(drive, intake, agitator, indexer, shooter, driverController, climber, led));

    // ==========================================
    // 2. DEFAULT DRIVE COMMAND IMPLEMENTATION
    // ==========================================
        indexer.setDefaultCommand(new RunCommand(() -> indexer.autoIndex(0.5), indexer));
    // Default commands run continuously when no other command claims the subsystem.
    drive.setDefaultCommand(new RunCommand(
        () -> {
            String mode = driveModeChooser.getSelected();
            if (mode == null) mode = "RACING"; 

            switch (mode) {
                case "TANK":
                    double leftTank = MathUtil.applyDeadband(-driverController.getRawAxis(1), 0.05);
                    double rightTank = MathUtil.applyDeadband(-driverController.getRawAxis(getRightStickYAxis()), 0.05);
                    drive.tankDrive(leftTank, rightTank);
                    break;

                case "RACING":
                default:
                    // Read controller axes and apply deadbands to eliminate stick drift
                    double throttle = MathUtil.applyDeadband(getForwardSpeed(), 0.02);
                    double steer = MathUtil.applyDeadband(getSteeringSpeed(), 0.02);

                    if (isDemoMode) {
                        throttle *= 0.35; // Cap forward/reverse speed to 35%
                        steer *= 0.35;    // Cap turn speed to 35%
                    }

                    // Read Right Stick Aim-Bot input (50% deadzone)
                    double rightStickX = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickXAxis()), 0.50);
                    double rightStickY = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickYAxis()), 0.50);

                    // 🔗 CONNECTOR: Here is where controller values are passed to Drive.java methods!
                    if (Math.abs(rightStickX) > 0 || Math.abs(rightStickY) > 0) {
                        double targetAngle = Math.toDegrees(Math.atan2(-rightStickX, rightStickY));
                        drive.snapToAngleDrive(throttle, targetAngle);
                    } else {
                        // Pass 'throttle' and 'steer' as inputs into Drive.java's arcadeDrive method!
                        drive.arcadeDrive(throttle, steer);
                    }
                    break;
            }
        }, 
        drive // Requirement: Locks ownership of the Drive subsystem
    ));

    configureButtonBindings();
  }

  // ==========================================
  //          SMART AXIS / CONTROLLER MATH
  // ==========================================
  private boolean isXboxMode() {
      String selected = controllerChooser.getSelected();
      if (selected != null && !selected.equals("AUTO")) {
          return selected.equals("XBOX");
      }

      if (cachedXboxMode != null) {
          return cachedXboxMode;
      }

      String controllerName = DriverStation.getJoystickName(0);
      if (controllerName.trim().isEmpty()) {
          return false; 
      }

      boolean isXbox = controllerName.toLowerCase().contains("xbox");
      cachedXboxMode = isXbox;
      System.out.println(">>> CONTROLLER LATCHED: " + controllerName + " | Xbox Mode: " + cachedXboxMode);
      return cachedXboxMode;
  }

  private double getForwardSpeed() {
      if (isXboxMode()) {
          // Xbox: Right Trigger (Axis 3) minus Left Trigger (Axis 2)
          return driverController.getRawAxis(3) - driverController.getRawAxis(2); 
      } else {
          // GameCube: Right Trigger (Axis 4) minus Left Trigger (Axis 3)
          return (driverController.getRawAxis(4) - driverController.getRawAxis(3)) / 2.0;
      }
  }

  private double getSteeringSpeed() {
      return driverController.getRawAxis(0); // Left Stick X
  }

  private int getRightStickXAxis() {
      return isXboxMode() ? 4 : 2; 
  }

  private int getRightStickYAxis() {
      return isXboxMode() ? 5 : 5; 
  }
 

  // ==========================================
  //          BUTTON BINDINGS
  // ==========================================
  private void configureButtonBindings() {

      SmartDashboard.putBoolean("Run System Check", false);
      NetworkButton systemCheckToggle = new NetworkButton("SmartDashboard", "Run System Check");

      // System Check Trigger with Cleanup Crew (.finallyDo)
      systemCheckToggle.whileTrue(
          new SystemCheck(drive, intake, agitator, indexer, shooter, driverController, climber, led)
          .finallyDo((boolean interrupted) -> {
              drive.restoreStandardDriving(); 
              intake.stop();
              agitator.stop();
              indexer.stop();
              shooter.stop();
              climber.stop();
              
              SmartDashboard.putBoolean("Run System Check", false);
              
              if (interrupted) {
                  SmartDashboard.putString("TEST STATUS", "❌ TEST ABORTED.");
                  RobotContainer.led.setTestFailed(); 
              } else {
                  RobotContainer.led.clearTestMode(); 
              }
          })
      );

      // Gatekeeper: Mutes normal driver buttons during a System Check
      Trigger isNotTesting = new Trigger(() -> !SmartDashboard.getBoolean("Run System Check", false));

      // Gyro Zero (Button 8 / Start)
      JoystickButton zeroGyroButton = new JoystickButton(driverController, 8);
      zeroGyroButton.onTrue(new InstantCommand(() -> drive.zeroHeading(), drive));

      // Controller Haptic Rumble when holding 3+ Cargo
      new Trigger(() -> indexer.getCargoCount() >= 3)
          .onTrue(new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
          .onFalse(new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));

      // A BUTTON (1) -> INTAKE TOGGLE
      JoystickButton intakeButton = new JoystickButton(driverController, 1);
      intakeButton.and(isNotTesting).toggleOnTrue(
          new RunCommand(() -> {
              intake.setSpeed(1.0);
              agitator.setSpeed(0.5);
             // indexer.autoIndex(0.5);
              intake.deploy();
          }, intake, agitator).finallyDo(() -> {
              intake.stop();
              agitator.stop();
             // indexer.autoIndex(0.5);
              intake.resetLockout();
          })
      );

      // X BUTTON (3) -> OUTTAKE / EJECT
      JoystickButton buttonX = new JoystickButton(driverController, 3);
      buttonX.and(isNotTesting).whileTrue(new RunCommand(() -> {
          intake.deploy();
          intake.setSpeed(-0.8);
          agitator.setSpeed(-0.6);
          indexer.setSpeed(-0.5);
      }, intake, agitator, indexer)
      ).onTrue(
          new InstantCommand(() -> indexer.cancelStaging())
      ).onFalse(new RunCommand(() -> {
          intake.stop();
          agitator.stop();
          indexer.stop();
      }, intake, agitator, indexer));

      // B BUTTON (2) -> LOW SPEED SHOT
      JoystickButton buttonB = new JoystickButton(driverController, 2);
      buttonB.and(isNotTesting).whileTrue(new RunCommand(() -> {
      double liveSpeed = SmartDashboard.getNumber("Tuning/Low Shoot Speed", 0.38);
          
          shooter.setSpeed(liveSpeed);
          indexer.setSpeed(1.0);
          agitator.setSpeed(1.0);
      }, shooter, indexer, agitator)).onFalse(new RunCommand(() -> {
          shooter.stop();
          indexer.stop();
          agitator.stop();
      }, shooter, indexer, agitator));

      // Y BUTTON (4) -> HIGH SPEED FIRE
      JoystickButton highFireButton = new JoystickButton(driverController, 4);
      highFireButton.and(isNotTesting).whileTrue(new RunCommand(() -> {
          if (!isDemoMode) {
              shooter.setSpeed(1.0); 
              indexer.setSpeed(1.0);
              agitator.setSpeed(1.0);
          }
      }, shooter, indexer, agitator)).onFalse(new RunCommand(() -> {
          shooter.stop();
          indexer.stop();
          agitator.stop();
      }, shooter, indexer, agitator));

      // LEFT BUMPER (5) -> AUTO AIM
      JoystickButton leftBumper = new JoystickButton(driverController, 5);
      leftBumper.and(isNotTesting).whileTrue(new RunCommand(() -> {
          drive.autoAim(limelight, 0.0);
      }, drive)).onFalse(new InstantCommand(() -> {
          drive.stop();
          limelight.setLEDOff(); 
      }, drive));

      // LEFT STICK PRESS (9) -> RETRACT INTAKE
      JoystickButton leftStickButton = new JoystickButton(driverController, 9);
      leftStickButton.and(isNotTesting).whileTrue(new RunCommand(() -> {
          intake.retract();
      }, intake)).onFalse(new InstantCommand(() -> {
      }, intake));

      // RIGHT STICK PRESS (10) -> TOGGLE CLIMBER ARMS
      new JoystickButton(driverController, 10).and(isNotTesting)
          .onTrue(new InstantCommand(() -> climber.toggleArms(), climber));

      // BACK / SELECT (7) -> DEMO MODE TOGGLE
      JoystickButton demoModeButton = new JoystickButton(driverController, 7);
      demoModeButton.onTrue(new InstantCommand(() -> {
          isDemoMode = !isDemoMode; 
          SmartDashboard.putBoolean("DEMO MODE ACTIVE", isDemoMode);
      }));

// RIGHT BUMPER (6) -> FULL AUTO-SCORE SEQUENCE
      JoystickButton rightBumper = new JoystickButton(driverController, 6);
      rightBumper.and(isNotTesting).whileTrue(new RunCommand(() -> {
          drive.autoAim(limelight, 0.0);
          if (limelight.isCentered(2)) { 
              
              // GET BOTH PIECES OF DATA FROM THE LIMELIGHT
              double currentDistance = limelight.getDistanceToTarget();
              double currentTag = limelight.getTargetID();
              
              // SEND BOTH TO THE SHOOTER
              shooter.setSpeedForDistance(currentDistance, currentTag);
              
              if (shooter.isReadyToFire()) {
                  indexer.setSpeed(1.0); 
                  agitator.setSpeed(0.8); 
              } else {
                  indexer.stop(); 
                  agitator.stop();
              }
          } else {
              shooter.stop(); 
              indexer.stop();
              agitator.stop();
          }
      }, drive, shooter, indexer, agitator)).onFalse(new InstantCommand(() -> {
          drive.stop();
          shooter.stop();
          indexer.stop();
          agitator.stop();
          limelight.setLEDOff(); 
      }, drive, shooter, indexer, agitator));

      // CLIMBER CONTROLS (D-Pad)
      new Trigger(() -> driverController.getPOV() == 0).and(isNotTesting)
          .whileTrue(new RunCommand(() -> climber.setBoth(1.0), climber))
          .onFalse(new RunCommand(() -> climber.stop(), climber));

      new Trigger(() -> driverController.getPOV() == 180).and(isNotTesting)
          .whileTrue(new RunCommand(() -> climber.setBoth(-1.0), climber))
          .onFalse(new RunCommand(() -> climber.stop(), climber));

      new Trigger(() -> driverController.getPOV() == 270).and(isNotTesting)
          .whileTrue(new RunCommand(() -> climber.setLeft(1.0), climber))
          .onFalse(new RunCommand(() -> climber.stop(), climber));

      new Trigger(() -> driverController.getPOV() == 90).and(isNotTesting)
          .whileTrue(new RunCommand(() -> climber.setRight(1.0), climber))
          .onFalse(new RunCommand(() -> climber.stop(), climber));
  }
}