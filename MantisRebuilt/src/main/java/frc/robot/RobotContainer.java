package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
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
@SuppressWarnings("unused")
public class RobotContainer {
  
  public static final Drive drive = new Drive();
  public static final Intake intake = new Intake();
  public static final Agitator agitator = new Agitator();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  public static final Climber climber = new Climber();
  public static final LED led = new LED(indexer, intake, shooter); 
  public static final Limelight limelight = new Limelight();

  private final Joystick driverController = new Joystick(0);
// We use 'Boolean' (capital B) so it can start as 'null' (unknown)
  private Boolean cachedXboxMode = null;
// --- SAFETY STATE ---
  public static boolean isDemoMode = false;

  // --- DASHBOARD CHOOSERS ---
  private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
  private final SendableChooser<String> controllerChooser = new SendableChooser<>();

  public RobotContainer() {
    // 1. Controller Chooser Setup
    controllerChooser.setDefaultOption("Auto-Detect Controller", "AUTO");
    controllerChooser.addOption("Force Xbox Mode", "XBOX");
    controllerChooser.addOption("Force GameCube Mode", "GAMECUBE");
    SmartDashboard.putData("Controller Type", controllerChooser);
    // 2. Drive Mode Chooser Setup
    driveModeChooser.setDefaultOption("Racing Drive (Triggers + Aim-Bot)", "RACING");
    driveModeChooser.addOption("Tank Drive (Left Y, Right Y)", "TANK");
    SmartDashboard.putData("Drive Mode", driveModeChooser);
    // Puts a clickable "Play" button directly on Shuffleboard!
    SmartDashboard.putData("Run Mantis System Check", 
        new SystemCheck(drive, intake, agitator, indexer, shooter, driverController, climber));

    // --- DYNAMIC DEFAULT DRIVE COMMAND ---
    drive.setDefaultCommand(new RunCommand(
        () -> {
            String mode = driveModeChooser.getSelected();
            if (mode == null) mode = "RACING"; 

            switch (mode) {
                case "TANK":
                    double leftTank = MathUtil.applyDeadband(-driverController.getRawAxis(1), 0.1);
                    double rightTank = MathUtil.applyDeadband(-driverController.getRawAxis(getRightStickYAxis()), 0.1);
                    drive.tankDrive(leftTank, rightTank);
                    break;

                case "RACING":
                default:
                    // Normal driving inputs
                    double throttle = MathUtil.applyDeadband(getForwardSpeed(), 0.05);
                    double steer = MathUtil.applyDeadband(getSteeringSpeed(), 0.1);
                  // --- DEMO MODE SPEED CAP ---
                    if (isDemoMode) {
                        throttle *= 0.35; // Hard cap forward/reverse to 35%
                        steer *= 0.35;    // Hard cap turning to 35%
                    }
                    // --- NEW 50% DEADZONE ON RIGHT STICK ---
                    double rightStickX = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickXAxis()), 0.50);
                    double rightStickY = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickYAxis()), 0.50);

                    // If the stick is pushed PAST the 50% deadzone, use the Aim-Bot
                    if (Math.abs(rightStickX) > 0 || Math.abs(rightStickY) > 0) {
                        double targetAngle = Math.toDegrees(Math.atan2(-rightStickX, rightStickY));
                        drive.snapToAngleDrive(throttle, targetAngle);
                    } else {
                        // Otherwise, normal left-stick steering
                        drive.arcadeDrive(throttle, steer);
                    }
                    break;
            }
        }, 
        drive
    ));

    configureButtonBindings();
  }

  // --- SMART AXIS MATH ---
private boolean isXboxMode() {
      // 1. Dashboard Override checks first (Extremely fast, zero string manipulation)
      String selected = controllerChooser.getSelected();
      if (selected != null && !selected.equals("AUTO")) {
          return selected.equals("XBOX");
      }

      // 2. THE LATCH: If we successfully figured out the controller earlier, 
      // just return the saved answer. This uses literally zero CPU!
      if (cachedXboxMode != null) {
          return cachedXboxMode;
      }

      // 3. If we are here, we don't know the controller yet. Ask the Driver Station.
      String controllerName = DriverStation.getJoystickName(0);
      
      // 4. The Boot-Up Bug: If the string is empty, the laptop hasn't connected yet.
      // Return a temporary safe value (false), but DON'T save it so it tries again next loop.
      if (controllerName.trim().isEmpty()) {
          return false; 
      }

      // 5. The laptop finally connected! Figure out what controller it is...
      boolean isXbox = controllerName.toLowerCase().contains("xbox");

      // ...and permanently latch it into the cache!
      cachedXboxMode = isXbox;
      
      // Print a one-time confirmation to the driver station console
      System.out.println(">>> CONTROLLER LATCHED: " + controllerName + " | Xbox Mode: " + cachedXboxMode);
      
      return cachedXboxMode;
  }

  private double getForwardSpeed() {
      if (isXboxMode()) {
          // Xbox: RT is Axis 3, LT is Axis 2 (Values 0.0 to 1.0)
          double rightTrigger = driverController.getRawAxis(3); 
          double leftTrigger = driverController.getRawAxis(2);  
          return rightTrigger - leftTrigger; 
      } else {
          // GameCube: RT is Axis 4, LT is Axis 3 (Values -1.0 to 1.0)
          // Simplified math to fix the inverted driving issue
          double rightTrigger = driverController.getRawAxis(4); 
          double leftTrigger = driverController.getRawAxis(3);  
          return (rightTrigger - leftTrigger) / 2.0;
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

  // --- BUTTON BINDINGS ---
private void configureButtonBindings() {

// Initialize the toggle on the dashboard so it shows up the moment the RIO boots
SmartDashboard.putBoolean("Run System Check", false);

// Bind the dashboard widget directly to your command sequence
NetworkButton systemCheckToggle = new NetworkButton("SmartDashboard", "Run System Check");

systemCheckToggle
    .onTrue(new SystemCheck(drive, intake, agitator, indexer, shooter, driverController, climber))
    .onFalse(new InstantCommand(() -> {
        // FAILSAFE: If you click the toggle OFF mid-test, aggressively stop everything!
        drive.stop();
        intake.stop();
        agitator.stop();
        indexer.stop();
        shooter.stop();
        climber.stop();
        SmartDashboard.putString("TEST STATUS", "X TEST ABORTED BY DASHBOARD.");
    }));


      // Gyro Zero (Button 8 / Start)
      JoystickButton zeroGyroButton = new JoystickButton(driverController, 8);
      zeroGyroButton.onTrue(new InstantCommand(() -> drive.zeroHeading(), drive));

      // Haptic Rumble for 3+ Cargo
      new Trigger(() -> indexer.getCargoCount() >= 3)
          .onTrue(new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
          .onFalse(new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));

      // A BUTTON (1) -> INTAKE
      JoystickButton intakeButton = new JoystickButton(driverController, 1);
     intakeButton.toggleOnTrue(
          new RunCommand(() -> {
              // What happens while toggled ON:
              intake.setSpeed(0.5);
              agitator.setSpeed(0.5);
            indexer.autoIndex(0.5);
              
          }, intake, agitator).finallyDo(() -> {
              // What happens the exact moment you toggle it OFF (or if it gets interrupted):
              intake.stop();
              agitator.stop();
            indexer.autoIndex(0.5);
              // Reset the comm-loss latch so you can use it again!
              intake.resetLockout();
          })
      );
   /*    buttonA.whileTrue(new RunCommand(() -> {
          intake.deploy();
          intake.setSpeed(0.8);
          agitator.setSpeed(0.6);
          indexer.autoIndex(0.5);
      }, intake, agitator, indexer)).onFalse(new RunCommand(() -> {
          intake.stop();
          agitator.stop();
          indexer.autoIndex(0.5);
      }, intake, agitator, indexer));
*/
      // X BUTTON (3) -> OUTTAKE / EJECT
      JoystickButton buttonX = new JoystickButton(driverController, 3);
      buttonX.whileTrue(new RunCommand(() -> {
                  intake.deploy();
          intake.setSpeed(-0.8);
          agitator.setSpeed(-0.6);
          indexer.setSpeed(-0.5);
      }, intake, agitator, indexer)
      ).onTrue(
          // THE FIX: The moment you press Outtake, forcefully cancel auto-staging!
          new InstantCommand(() -> indexer.cancelStaging())
          ).onFalse(new RunCommand(() -> {
          intake.stop();
          agitator.stop();
          indexer.stop();
      }, intake, agitator, indexer));

      // B BUTTON (2) -> LOW SPEED SHOT
      JoystickButton buttonB = new JoystickButton(driverController, 2);
      buttonB.whileTrue(new RunCommand(() -> {
          shooter.setSpeed(0.3); // Adjust this decimal to find the perfect low shot!
          indexer.setSpeed(1.0);
          agitator.setSpeed(1.0);
      }, shooter, indexer, agitator)).onFalse(new RunCommand(() -> {
          shooter.stop();
          indexer.stop();
          agitator.stop();
      }, shooter, indexer, agitator));

      // --- HIGH SPEED FIRE (Y Button - 4) ---
      JoystickButton highFireButton = new JoystickButton(driverController, 4);
      
      highFireButton.whileTrue(new RunCommand(() -> {
          // Only allow the high-speed shot if the safety is OFF
          if (!isDemoMode) {
              shooter.setSpeed(1.0); // 100% Power
              indexer.setSpeed(1.0);
              agitator.setSpeed(1.0);
          }
      }, shooter, indexer, agitator)).onFalse(new RunCommand(() -> {
          shooter.stop();
          indexer.stop();
          agitator.stop();
      }, shooter, indexer, agitator));

        // LEFT BUMPER (5) -> REV SHOOTER (Forcing it to spin up backwards to clear jams or shoot backwards if we want to get crazy)
      JoystickButton leftBumper = new JoystickButton(driverController, 5);
      
      leftBumper.whileTrue(new RunCommand(() -> {
          drive.autoAim(limelight, 0.0);
      }, drive)).onFalse(new RunCommand(() -> {
          drive.stop();
      }, drive));

      //-- Left Stick Press for retracting the intake
        JoystickButton leftStickButton = new JoystickButton(driverController, 9);

        leftStickButton.whileTrue(new RunCommand(() -> {
            intake.retract();
        }, intake)).onFalse(new RunCommand(() -> {
            // Do nothing on release, just let it stay retracted until we want to deploy it again with the A button
        }, intake));
// -- RIght Stick press for toggling the arms
new JoystickButton(driverController, 10) // Port 9 is usually Right Stick Press
      .onTrue(new InstantCommand(() -> climber.toggleArms(), climber));
// --- DEMO MODE TOGGLE (Button 7 - "Back/Select") ---
      // Pressing this flips the mode between True and False instantly
      JoystickButton demoModeButton = new JoystickButton(driverController, 7);
      
      demoModeButton.onTrue(new InstantCommand(() -> {
          isDemoMode = !isDemoMode; // Flip the switch
          
          // Print it to the driver station so the coach knows it is safe!
          SmartDashboard.putBoolean("DEMO MODE ACTIVE", isDemoMode);
      }));
// RIGHT BUMPER (6) -> THE FULL AUTO-SCORE SEQUENCE
      JoystickButton rightBumper = new JoystickButton(driverController, 6);
      
      rightBumper.whileTrue(new RunCommand(() -> {
          // STEP 1: The Drivetrain ALWAYS tries to aim
          drive.autoAim(limelight, 0.0);

          // GATEKEEPER 1: Are we centered?
          if (limelight.isCentered(2)) { 
              
              // We are centered! Read distance and spool the flywheel!
              double currentDistance = limelight.getDistanceToTarget();
              shooter.setSpeedForDistance(currentDistance);

              // GATEKEEPER 2: Is the flywheel up to speed?
              if (shooter.isReadyToFire()) {
                  // WE ARE CENTERED AND SPOOLED. FIRE!
                  intake.setSpeed(1.0);
                  indexer.setSpeed(1.0); 
                  agitator.setSpeed(0.8); // Keep the queue moving!
              } else {
                  // Waiting for the flywheel to finish accelerating...
                  intake.stop();
                  indexer.stop(); 
                  agitator.stop();
              }

          } else {
              // We are NOT centered. Keep the shooter and indexer completely off!
              shooter.stop(); 
              indexer.stop();
              agitator.stop();
              intake.stop();
          }

      // We must require all 4 subsystems so they don't fight other buttons!
      }, drive, shooter, indexer, agitator,intake)).onFalse(new InstantCommand(() -> {
          
          // When you let go of the bumper, shut the entire sequence down.
          drive.stop();
          shooter.stop();
          indexer.stop();
          agitator.stop();
          intake.stop();
          limelight.setLEDOff(); 
          
      }, drive, shooter, indexer, agitator,intake));
      // --- CLIMBER CONTROLS (D-Pad) ---
      new Trigger(() -> driverController.getPOV() == 0)
          .whileTrue(new RunCommand(() -> climber.setBoth(1.0), climber))
          .onFalse(new RunCommand(() -> climber.stop(), climber));

      new Trigger(() -> driverController.getPOV() == 180)
          .whileTrue(new RunCommand(() -> climber.setBoth(-1.0), climber))
          .onFalse(new RunCommand(() -> climber.stop(), climber));

      new Trigger(() -> driverController.getPOV() == 270)
          .whileTrue(new RunCommand(() -> climber.setLeft(1.0), climber))
          .onFalse(new RunCommand(() -> climber.setLeft(0), climber));

      new Trigger(() -> driverController.getPOV() == 90)
          .whileTrue(new RunCommand(() -> climber.setRight(1.0), climber))
          .onFalse(new RunCommand(() -> climber.setRight(0), climber));
  }
  public void periodic() {
      // --- NAVX AXIS TESTER ---
      SmartDashboard.putNumber("NavX YAW", drive.getYaw());

  }
}