// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  
  public static final Drive drive = new Drive();
  private final Joystick driverController = new Joystick(0);
  private boolean isXboxController = false;
  public static final Intake intake = new Intake();
  public static final Agitator agitator = new Agitator();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  // --- THE DRIVE MODE CHOOSER ---
  private final SendableChooser<String> driveModeChooser = new SendableChooser<>();

  public RobotContainer() {
    // Controller Detection
    String controllerName = DriverStation.getJoystickName(0);
    if (controllerName.toLowerCase().contains("xbox")) {
        isXboxController = true;
    }

    // Setup the Dashboard Dropdown
    driveModeChooser.setDefaultOption("Racing Drive (Triggers + Aim-Bot)", "RACING");
    driveModeChooser.addOption("Split Arcade (Left Y, Right X)", "ARCADE");
    driveModeChooser.addOption("Tank Drive (Left Y, Right Y)", "TANK");
    SmartDashboard.putData("Drive Mode", driveModeChooser);

    // --- DYNAMIC DEFAULT DRIVE COMMAND ---
    drive.setDefaultCommand(new RunCommand(
        () -> {
            // Get the currently selected drive mode from the dashboard
            String mode = driveModeChooser.getSelected();
            if (mode == null) mode = "RACING"; // Failsafe

            switch (mode) {
                case "ARCADE":
                    // Left Stick Y (Inverted because forward is negative on joysticks)
                    double arcadeThrottle = MathUtil.applyDeadband(-driverController.getRawAxis(1), 0.1);
                    double arcadeSteer = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickXAxis()), 0.1);
                    drive.arcadeDrive(arcadeThrottle, arcadeSteer);
                    break;

                case "TANK":
                    // Left Stick Y controls Left side, Right Stick Y controls Right side
                    double leftTank = MathUtil.applyDeadband(-driverController.getRawAxis(1), 0.1);
                    double rightTank = MathUtil.applyDeadband(-driverController.getRawAxis(getRightStickYAxis()), 0.1);
                    drive.tankDrive(leftTank, rightTank);
                    break;

                case "RACING":
                default:
                    // Read joysticks with deadbands to stop stick drift
                    double rightStickX = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickXAxis()), 0.15);
                    double rightStickY = MathUtil.applyDeadband(driverController.getRawAxis(getRightStickYAxis()), 0.15);
                    double throttle = MathUtil.applyDeadband(getForwardSpeed(), 0.05);

                    // If right stick is pushed past the deadband, use Aim-Bot
                    if (Math.abs(rightStickX) > 0 || Math.abs(rightStickY) > 0) {
                        double targetAngle = Math.toDegrees(Math.atan2(rightStickX, -rightStickY));
                        drive.snapToAngleDrive(throttle, targetAngle);
                    } else {
                        // Otherwise, drive normally with Left Stick steering
                        double steer = MathUtil.applyDeadband(getSteeringSpeed(), 0.1);
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
  private double getForwardSpeed() {
      if (isXboxController) {
          double rightTrigger = driverController.getRawAxis(3); 
          double leftTrigger = driverController.getRawAxis(2);  
          return rightTrigger - leftTrigger; 
      } else {
          double rightTrigger = driverController.getRawAxis(4); 
          double leftTrigger = driverController.getRawAxis(3);  
          return ((1 - leftTrigger) - (1 - rightTrigger)) / 2;
      }
  }

  private double getSteeringSpeed() {
      return driverController.getRawAxis(0); // Left Stick X
  }

  private int getRightStickXAxis() {
      return isXboxController ? 4 : 2; 
  }

  private int getRightStickYAxis() {
      return isXboxController ? 5 : 5; 
  }

  // --- BUTTON BINDINGS ---
private void configureButtonBindings() {
      // Button 8 (Start) Zeroes Gyro
      JoystickButton zeroGyroButton = new JoystickButton(driverController, 8);
      zeroGyroButton.onTrue(new RunCommand(() -> drive.zeroHeading(), drive).withTimeout(0.1));

      // Button 1 (A on Xbox) -> INTAKE
      JoystickButton intakeButton = new JoystickButton(driverController, 1);
      intakeButton.whileTrue(new RunCommand(() -> {
          intake.deploy();
          intake.setSpeed(0.8);
          agitator.setSpeed(0.6);
          indexer.setSpeed(0.5);
      }, intake, agitator, indexer)).onFalse(new RunCommand(() -> {
          intake.retract();
          intake.stop();
          agitator.stop();
          indexer.stop();
      }, intake, agitator, indexer));

      // Button 6 (Right Bumper) -> SHOOT
      JoystickButton shootButton = new JoystickButton(driverController, 6);
      shootButton.whileTrue(new RunCommand(() -> {
          shooter.setSpeed(0.9);
          indexer.setSpeed(1.0);
          agitator.setSpeed(1.0);
      }, shooter, indexer, agitator)).onFalse(new RunCommand(() -> {
          shooter.stop();
          indexer.stop();
          agitator.stop();
      }, shooter, indexer, agitator));

      // Button 2 (B on Xbox) -> EJECT
      JoystickButton ejectButton = new JoystickButton(driverController, 2);
      ejectButton.whileTrue(new RunCommand(() -> {
          intake.setSpeed(-0.8);
          agitator.setSpeed(-0.6);
          indexer.setSpeed(-0.5);
      }, intake, agitator, indexer)).onFalse(new RunCommand(() -> {
          intake.stop();
          agitator.stop();
          indexer.stop();
      }, intake, agitator, indexer));
  }
  public Command getAutonomousCommand() {
      return null;
    // An example command will be run in autonomous
   
  }
}


  

