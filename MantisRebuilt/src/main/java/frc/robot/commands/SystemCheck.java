package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick; // Import Joystick!
import frc.robot.subsystems.*;

public class SystemCheck extends SequentialCommandGroup {

    // Pass the Joystick in through the constructor!
    public SystemCheck(Drive drive, Intake intake, Agitator agitator, Indexer indexer,Shooter shooter, Joystick driverController, Climber climber) {
        
        addCommands(
            // --- STEP 1: INITIALIZE ---
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "TESTING INTAKE...")),
            new InstantCommand(() -> intake.deploy()),

            // --- STEP 2: MANUAL CONFIRMATION (A or B) ---
            // Wait until the driver presses A (Port 1) or B (Port 2)
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(1) || driverController.getRawButtonPressed(2) || indexer.intakeDeploySensor.get()),
            
            // Check which button was pressed
            new InstantCommand(() -> {
                if (driverController.getRawButtonPressed(2)) { // If B was pressed
                    SmartDashboard.putString("TEST STATUS", "x INTAKE DEPLOY FAILED");
                    this.cancel(); // Abort the whole sequence!
                } else {
                    SmartDashboard.putString("TEST STATUS", " INTAKE PASSED. FEED CARGO NOW.");
                }
            }),


            // --- STEP 3: THE AUTO-TRACKING CARGO PATH ---
            // Turn on the intake and agitator
            new InstantCommand(() -> {
                intake.setSpeed(0.5);
                agitator.setSpeed(0.5);
            }),
            
            // Wait until the physical entry sensor trips!
            new WaitUntilCommand(() -> indexer.entrySensor.get() == false),
            
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "ENTRY SENSOR GOOD. TESTING INDEXER.")),

            // Hand it off to the indexer...
            new InstantCommand(() -> {
               
                indexer.setSpeed(0.3);
            }),

            // Wait until it hits the middle sensor!
            new WaitUntilCommand(() -> indexer.middleSensor.get() == false),
            
            new InstantCommand(() -> {
                 intake.stop();
                 indexer.stop();
                SmartDashboard.putString("TEST STATUS", "MIDDLE SENSOR GOOD. TESTING SHOOTER EXIT.");
            }),
            
            // ... You just keep chaining these together for the Shooter and Climber!
            // ... (Your previous code testing the Middle Sensor) ...

            // --- STEP 4: SHOOTER SENSOR TEST ---
            // Inch the cargo upward just enough to trip the exit sensor
            new InstantCommand(() -> {
                agitator.stop();
                shooter.setSpeed(0.26); // Spin very slowly so we don't accidentally shoot!
                indexer.setSpeed(0.2);
            }),

            new WaitUntilCommand(() -> indexer.exitSensor.get() == false),

            new InstantCommand(() -> {
                indexer.stop();
                shooter.stop();
                SmartDashboard.putString("TEST STATUS", "EXIT SENSOR GOOD. CLEAR CARGO THEN PRESS 'A'.");
            }),

            // --- STEP 5: FLYWHEEL RPM TEST ---
            // Wait for A Button to ensure hands are out of the shooter!
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(1)),
            
            new InstantCommand(() -> {
                SmartDashboard.putString("TEST STATUS", "SPOOLING FLYWHEEL...");
                shooter.setSpeed(0.8);
            }),

            // Your custom method acting as a gatekeeper!
            new WaitUntilCommand(() -> shooter.isReadyToFire()),

            new InstantCommand(() -> {
                shooter.stop();
                SmartDashboard.putString("TEST STATUS", "✅ FLYWHEEL AT SPEED. TESTING CLIMBERS.");
            }),

            // --- STEP 6: CLIMBER TEST ---
            new InstantCommand(() -> climber.setBoth(0.3)),
            new InstantCommand(() -> climber.toggleArms()),
            
            new WaitCommand(2.0), // Drive climbers up for 1 second
            
            new InstantCommand(() -> climber.setBoth(-0.3)),
            new InstantCommand(() -> climber.toggleArms()),

            new WaitCommand(2.0), // Drive climbers back down for 1 second

            new InstantCommand(() -> {
                climber.stop();
                SmartDashboard.putString("TEST STATUS", "Visual check Climbers OK?");
            }),
    
 // --- STEP 7: GEARBOX LINKAGE TEST ---
            new InstantCommand(() -> SmartDashboard.putString("TEST STATUS", "CLEAR WHEELS. PRESS 'X' TO TEST GEARBOX.")),
            new WaitUntilCommand(() -> driverController.getRawButtonPressed(3)),
            
            // 7A: Front Drives Back
            new InstantCommand(() -> {
                SmartDashboard.putString("TEST STATUS", "TEST 1: FRONT DRIVING BACK...");
                drive.testFrontDrivesBack(0.3); // Upped to 30% power to overcome static friction
            }),
            
            // THE FIX: Wait until BOTH are spinning, but give up after 3 seconds!
            new WaitUntilCommand(() -> drive.isLeftBackSpinning() && drive.isRightBackSpinning())
                .withTimeout(3.0),
            
            // Check the results to isolate the mechanical failure
            new InstantCommand(() -> {
                boolean leftGood = drive.isLeftBackSpinning();
                boolean rightGood = drive.isRightBackSpinning();
                
                if (leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "FRONT->BACK LINKAGE GOOD.");
                    drive.restoreStandardDriving(); 
                } else if (leftGood && !rightGood) {
                    SmartDashboard.putString("TEST STATUS", "X RIGHT GEARBOX BIND.");
                    drive.restoreStandardDriving();
                    this.cancel(); // Abort!
                } else if (!leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "X LEFT GEARBOX BIND.");
                    drive.restoreStandardDriving();
                    this.cancel(); // Abort!
                } else {
                    SmartDashboard.putString("TEST STATUS", "X TOTAL REAR LINKAGE FAILURE.");
                    drive.restoreStandardDriving();
                    this.cancel(); // Abort!
                }
            }),
            
            new WaitCommand(1.0), // Let the wheels coast down for a second
            
            // 7B: Back Drives Front
            new InstantCommand(() -> {
                SmartDashboard.putString("TEST STATUS", "TEST 2: BACK DRIVING FRONT...");
                drive.testBackDrivesFront(0.3); // Upped to 30% power
            }),
            
            new WaitUntilCommand(() -> drive.isLeftFrontSpinning() && drive.isRightFrontSpinning())
                .withTimeout(3.0),

            new InstantCommand(() -> {
                boolean leftGood = drive.isLeftFrontSpinning();
                boolean rightGood = drive.isRightFrontSpinning();
                
                if (leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "BACK->FRONT LINKAGE GOOD.");
                } else if (leftGood && !rightGood) {
                    SmartDashboard.putString("TEST STATUS", "X RIGHT GEARBOX BIND.");
                    drive.restoreStandardDriving();
                    this.cancel();
                } else if (!leftGood && rightGood) {
                    SmartDashboard.putString("TEST STATUS", "X LEFT GEARBOX BIND.");
                    drive.restoreStandardDriving();
                    this.cancel();
                } else {
                    SmartDashboard.putString("TEST STATUS", "X TOTAL FRONT LINKAGE FAILURE.");
                    drive.restoreStandardDriving();
                    this.cancel();
                }
            }),

            // --- SEQUENCE COMPLETE ---
            new InstantCommand(() -> {
                drive.restoreStandardDriving();
                SmartDashboard.putBoolean("Run System Check", false); 
                SmartDashboard.putString("TEST STATUS", "MANTIS IS FULLY READY FOR ACTION!");
            })
        ); // End of addCommands
    }
}
   