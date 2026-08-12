package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick; 
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;



// ==========================================
    //   COMMAND-BASED PROGRAMMING GLOSSARY
    // ==========================================
    // This file uses WPILib's "Command-Based" framework. Instead of writing one massive 
    // block of code, we snap smaller commands together like Lego bricks.
    //
    // SequentialCommandGroup: 
    // Think of this like a recipe or a playlist. It runs commands one by one, from top to bottom. 
    // It will NEVER move to the next command until the current one is completely finished.
    //
    // InstantCommand: 
    // A command that does its job in exactly one frame (0.02 seconds) and immediately finishes. 
    // It is perfect for "fire-and-forget" actions like flipping a switch, changing a variable, 
    // updating the dashboard, or turning a motor on.
    //
    // WaitUntilCommand: 
    // The "Gatekeeper." This completely freezes the sequence until the statement inside the 
    // parentheses becomes 'true'. We use this to wait for a sensor to trip or a button to be pressed.
    //
    // WaitCommand: 
    // A simple stopwatch. It pauses the sequence for the requested number of seconds.



public class SystemCheck extends SequentialCommandGroup {

public SystemCheck(Drive drive, Intake intake, Agitator agitator, Indexer indexer, Shooter shooter, Joystick driverController, Climber climber, LED led) {
        
        // --- THE BOUNCER ---
        // addRequirements() tells the robot's brain, "This sequence owns these subsystems."
        // If normal driver controls try to use the Intake or Drive while this test is running, 
        // the robot will actively block the driver to prevent the code from fighting itself!
        addRequirements(drive, intake, agitator, indexer, shooter, climber, led);
        
        addCommands(
            // --- STEP 0: PROMPT & SETUP ---
            // Start the LED test mode to overwrite the natural LED patterns during the actions of the robot. you can find this method in the LED subsytem
            //then we wet up the prompt to begin the test, we don't have a cancel button here because we can simply shift the toggle off or disable the robot before anything happens, later on the fastest reaction might be with the controller thus we have a b to cancel option
            new InstantCommand(() -> RobotContainer.led.startTestMode()),
            new InstantCommand(() -> SmartDashboard.putBoolean("Run System Check", true)),
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "PRESS 'A' TO START SYSTEM CHECK.")),
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(1)), 
            
            // --- STEP 1: INITIALIZE ---
            // Reset the intake Lockout if we are enabling the robot and have not yet used the intake, this value was never reset in origonal versions of the code.
            //so the reset ensures even if we did a systems check before or failed to set the intake lockout, we are doing so now and the check will run properly.
            //after that we prompt the user incase of a failed deployment to cancel, that way if the pneumatics are disengaged or some other failure occurs we can stop it here
            new InstantCommand(() -> intake.resetLockout()),
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "TESTING INTAKE... if intake does not deploy, press 'B' to abort.")),
            new InstantCommand(() -> intake.setSpeed(0.6)), 

            // --- STEP 2: MANUAL CONFIRMATION ---
            //we wait for the userr to confirm before proceeding or if our sensor on the intake is working properly, it will automattically proceed
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(1) || driverController.getRawButtonPressed(2) || intake.isDeployed()),           
             //this command runs instatnly after we press a button or the sensor is tripped to ensure if we pressed b, we are still pressing it within the 50th of a second to trigger the cancel
            new InstantCommand(() -> {
                if (driverController.getRawButtonPressed(2)) { 
                    SmartDashboard.putString("TEST STATUS", "❌ INTAKE DEPLOY FAILED");
                    this.cancel(); 
                } else {
                    SmartDashboard.putString("TEST STATUS", "✅ INTAKE PASSED. FEED CARGO NOW.");
                    RobotContainer.led.setTestSuccess(); // Flash Green
                }
            }),
            new WaitCommand(1.5), // WaitCommand pauses long enough to get visual confirmation on robot and dashboard, not just a quick flicker of color or text and move on
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()), // Back to Yellow

            // --- STEP 3: AUTO-TRACKING CARGO PATH ---
            // the following steps track the cargo through the robot and instantly set motors and wait for each of the sensors trip on its way to the shooter as it clears at low velocity
            new InstantCommand(() -> agitator.setSpeed(0.5)),
            new WaitUntilCommand(() -> indexer.entrySensor.get() == false),
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "✅ ENTRY SENSOR GOOD. TESTING INDEXER.")),

            new InstantCommand(() -> indexer.setSpeed(0.6)),
            new WaitUntilCommand(() -> indexer.middleSensor.get() == false),
            
            new InstantCommand(() -> {
                 intake.stop();
                 indexer.stop();
                 SmartDashboard.putString("TEST STATUS", "✅ MIDDLE SENSOR GOOD. TESTING SHOOTER EXIT.");
                 RobotContainer.led.setTestSuccess();
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()),

            // ==========================================
            // --- STEP 4: SHOOTER SENSOR TEST ---
            // ==========================================
            
            // 1. AN INSTANT ACTION: We use an InstantCommand to quickly turn the motors on.
            // Because it finishes instantly, the sequence immediately moves to the next line 
            // while the motors continue to spin in the background.
            new InstantCommand(() -> {
                agitator.stop();
                shooter.setSpeed(0.26); 
                indexer.setSpeed(0.4);
            }),

            // 2. THE WAITING GAME: The sequence freezes right here. The motors are still running, 
            // but the code will not proceed until the ball physically trips the exit sensor.
            new WaitUntilCommand(() -> indexer.exitSensor.get() == false),

            // 3. THE CLEANUP: Once the sensor trips, the WaitUntilCommand finishes. 
            // We use another InstantCommand to shut the motors off and update the dashboard!
            new InstantCommand(() -> {
                indexer.stop();
                shooter.stop();
                SmartDashboard.putString("TEST STATUS", "✅ EXIT SENSOR GOOD.");
                RobotContainer.led.setTestSuccess();
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()),
            
            // --- STEP 5: FLYWHEEL RPM TEST ---
            // This section focuses on using the shooter commands to run a test on the flywheel and ensure its all connected and able to spin up to a decent amount of its max speed.
            new InstantCommand(() -> {
                SmartDashboard.putString("TEST STATUS", "SPOOLING FLYWHEEL...");
                shooter.setSpeed(0.8);
            }),
            new WaitUntilCommand(() -> shooter.isReadyToFire()),
            new InstantCommand(() -> {
                shooter.stop();
                SmartDashboard.putString("TEST STATUS", "✅ FLYWHEEL AT SPEED. TESTING CLIMBERS.");
                RobotContainer.led.setTestSuccess();
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()),

            // --- STEP 6: CLIMBER TEST ---
            // This section focuses on using the climber commands to run a test on the climbers and ensure its all connected and able to lift up and down properly.
            //notice the speeds at which the climbers rise up and down are different, this is because the climbers have springs that constantly force the arms up
            // so on the way up it is being assisted and travels much further than on the way down at the same power so we have to increase the poweer to get them 
            // fully retracted within a resonable time frame for the test. this is just a visual check though as we do not have sensors or encoders on the climber motors 
            new InstantCommand(() -> climber.setBoth(0.3)),
            new InstantCommand(() -> climber.toggleArms()),
            new WaitCommand(2.0), 
            
            new InstantCommand(() -> {
                climber.stop();
                SmartDashboard.putString("TEST STATUS", "Visual check Climbers lifted OK? Press 'Right Bumper' to pass or 'B' to abort.");
            }),
            //again this is how we run our basic controller confirmation, the buttons may move around the controller but it prevents mis presses or spamming through the check
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(6) || driverController.getRawButtonPressed(2)),
            new InstantCommand(() -> {
                if (driverController.getRawButtonPressed(2)) { 
                    SmartDashboard.putString("TEST STATUS", "❌ Climbers FAILED");
                    this.cancel(); 
                } else {
                    SmartDashboard.putString("TEST STATUS", "✅ Climbers PASSED.");
                    RobotContainer.led.setTestSuccess();
                }
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()),
            
            new InstantCommand(() -> climber.setBoth(-0.63)),
            new InstantCommand(() -> climber.toggleArms()),
            new WaitCommand(2.0), 

            new InstantCommand(() -> {
                climber.stop();
                SmartDashboard.putString("TEST STATUS", "Visual check Climbers retracted OK? Press 'Right Bumper' to pass or 'B' to abort.");
            }),
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(6) || driverController.getRawButtonPressed(2)),
            new InstantCommand(() -> {
                if (driverController.getRawButtonPressed(2)) { 
                    SmartDashboard.putString("TEST STATUS", "❌ Climbers FAILED");
                    this.cancel(); 
                } else {
                    SmartDashboard.putString("TEST STATUS", "✅ Climbers PASSED.");
                    RobotContainer.led.setTestSuccess();
                }
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()),
    
            // --- STEP 7: GEARBOX LINKAGE TEST ---
            //this one gets complicated, we run the wheel test by setting the motors to be independant and disable the follow features, then setting the braking to coast,
            //this allows us to run our system check commands in the drive.java file that is set up to check the motors spinning that are linked on the gearbox. 
            // essentially look at those commands and see, we turn on set of motors off let them free spin, and run the motors they are paired with, 
            // if the motor that is unpowered moved, then the linkage is good, if it does not move then the motors or linkage are disconnected or broken.
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "CLEAR WHEELS. PRESS 'Y' (4) TO TEST GEARBOX.")),
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(4)),
            
            // 7A: Front Drives Back
            new InstantCommand(() -> {
                SmartDashboard.putString("TEST STATUS", "TEST 1: FRONT DRIVING BACK...");
                drive.testFrontDrivesBack(0.3); 
            }),
            new WaitUntilCommand(() -> drive.isLeftBackSpinning() && drive.isRightBackSpinning())
                .withTimeout(3.0),
            new InstantCommand(() -> {
                boolean leftGood = drive.isLeftBackSpinning();
                boolean rightGood = drive.isRightBackSpinning();
                
                if (leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "✅ FRONT->BACK LINKAGE GOOD.");
                    drive.restoreStandardDriving(); 
                    RobotContainer.led.setTestSuccess();
                } else if (leftGood && !rightGood) {
                    SmartDashboard.putString("TEST STATUS", "❌ RIGHT GEARBOX BIND.");
                    drive.restoreStandardDriving();
                    this.cancel(); 
                } else if (!leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "❌ LEFT GEARBOX BIND.");
                    drive.restoreStandardDriving();
                    this.cancel(); 
                } else {
                    SmartDashboard.putString("TEST STATUS", "❌ TOTAL REAR LINKAGE FAILURE.");
                    drive.restoreStandardDriving();
                    this.cancel(); 
                }
            }),
            new WaitCommand(1.5),
            new InstantCommand(() -> RobotContainer.led.clearTestSuccess()),
            
            // 7B: Back Drives Front
            new InstantCommand(() -> {
                SmartDashboard.putString("TEST STATUS", "TEST 2: BACK DRIVING FRONT...");
                drive.testBackDrivesFront(0.3); 
            }),
            new WaitUntilCommand(() -> drive.isLeftFrontSpinning() && drive.isRightFrontSpinning())
                .withTimeout(3.0),
            new InstantCommand(() -> {
                boolean leftGood = drive.isLeftFrontSpinning();
                boolean rightGood = drive.isRightFrontSpinning();
                
                if (leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "✅ BACK->FRONT LINKAGE GOOD.");
                } else {
                    SmartDashboard.putString("TEST STATUS", "❌ TOTAL FRONT LINKAGE FAILURE.");
                    drive.restoreStandardDriving();
                    this.cancel();
                }
            }),

            // --- SEQUENCE COMPLETE ---
            //pat on back sequence, we display our ready state, and have the drivetrain chirp the zelda item song on the falcons, they are really quiet though. but hey it works.
            new InstantCommand(() -> {
                drive.restoreStandardDriving();
                SmartDashboard.putString("TEST STATUS", "✅ MANTIS IS FULLY READY FOR ACTION!");
                
               Drive.playSong("zelda.chrp"); 
                RobotContainer.led.setTestSuccess();
            }),
            
            // Wait 3 seconds to admire the flashing green lights and let the music play 
            // before the command officially finishes and hands control back to normal Teleop!
            new WaitCommand(3.0),
            // I have had the robot loose ability to drive before, but I think it was because while the audio is playing or "not stopped" the drive motors are not being updated, 
            // so I added a stop music command to ensure the audio is stopped and the drive motors are free to be updated again.
            new InstantCommand(() -> {
                Drive.stopMusic();
            })
        );
    }
}