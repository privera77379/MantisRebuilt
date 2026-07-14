package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;



public class Indexer extends SubsystemBase {
  private final TalonSRX indexTalon = new TalonSRX(4);
private final Timer debounceTimer = new Timer();
  // --- SENSORS ---
  // Update these ports to match your RoboRIO wiring!
  public final DigitalInput entrySensor = new DigitalInput(0); // Agitator sensor
  public final DigitalInput middleSensor = new DigitalInput(1); // Middle verification
  public final DigitalInput exitSensor = new DigitalInput(2); // Shooter plates sensor
  public final DigitalInput intakeDeploySensor = new DigitalInput(3); // intake lowered limit switch sensor

  // --- STATE VARIABLES ---
  private int cargoCount = 0;
  private boolean lastEntryState = false;
  private boolean lastExitState = false;
  private boolean lastMiddleState = false;
 private boolean trigger = false; // Your existing counting lockout
  private boolean isStaging = false; // NEW: Controls the auto-index motor
  private boolean intakeDeployedCheck = false; // NEW: Tracks if the intake is lowered

  public Indexer() {
    indexTalon.setInverted(false);
    debounceTimer.start();
  }

  public double getSpeed() {
      // Returns actual power from -1.0 to 1.0
      return indexTalon.getMotorOutputPercent();
  }
// Control methods for the indexer motor
  public void setSpeed(double speed) {
    indexTalon.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public int getCargoCount() {
    return cargoCount;
  }

public void autoIndex(double speed) {

      // 1. The Action
      // Run the motor ONLY if we have exactly 1 ball AND we are actively staging it
      if (getCargoCount() == 1 && isStaging) {
          indexTalon.set(TalonSRXControlMode.PercentOutput, speed);
      } else {
          indexTalon.set(TalonSRXControlMode.PercentOutput, 0);
      }
  }


  public void stop() {
    indexTalon.set(TalonSRXControlMode.PercentOutput, 0);
  }
// Utility method to get the current cargo count, which can be used by commands or displayed on the LED array and tracking system.
// Instantly aborts any automated indexing
  public void cancelStaging() {
      isStaging = false;
  }

  @Override
  public void periodic() {
      // ==========================================
      // 1. HARDWARE ABSTRACTION LAYER
      // ==========================================
      boolean currentEntry = !entrySensor.get(); 
      boolean currentMiddle = !middleSensor.get(); 
      boolean currentExit = !exitSensor.get(); 
      boolean intakeDeployedCheck = intakeDeploySensor.get();
    // --- ENTRANCE LOGIC ---
    // Grab the true physical direction of the cargo from the Agitator
// Get the live speed of the agitator to determine direction
      double agitatorSpeed = RobotContainer.agitator.getSpeed();

      // --- DIRECTION-AWARE ENTRY LOGIC ---
      if (agitatorSpeed > 0.1) {
          // WE ARE INTAKING (Moving Forward)
          // Rising Edge: The ball just pushed the limit switch UP.
          if (currentEntry && !lastEntryState) {
              cargoCount++; 
              isStaging = true; // Wake up the indexer motor!
          }
      } else if (agitatorSpeed < -0.1) {
          // WE ARE OUTTAKING (Moving Backward)
          // Falling Edge: The ball just finished spitting out and the arm dropped DOWN.
          if (!currentEntry && lastEntryState) {
              cargoCount--; 
          }
      }

      // --- THE HARD CEILING FAILSAFE ---
      // Stop staging if it hits the top sensor OR physically bumps the flywheel!
      if (currentExit || RobotContainer.shooter.isBeingBackdriven()) {
          isStaging = false;
      }

      
      // --- THE NORMAL AUTO-INDEX WATCHER ---
      if (isStaging) {
          // Falling Edge: Ball fully cleared the middle sensor to make room for Ball 2
          if (!currentMiddle && lastMiddleState) {
              isStaging = false; 
          }
      }

      // --- EXIT LOGIC (Shooting) ---
      // Falling Edge: Ball was touching shooter, now it's gone!
      if (!currentExit && lastExitState && debounceTimer.get() > 0.25) {
          cargoCount--;
          debounceTimer.reset();
      }
      // --- CLAMP THE COUNT ---
      // This immediately kills any random "ghost counting" bugs
      if (cargoCount < 0) cargoCount = 0;
      if (cargoCount > 3) cargoCount = 3; 

      // ==========================================
      // 3. SAVE STATES FOR NEXT LOOP
      // ==========================================
      lastEntryState = currentEntry;
      lastExitState = currentExit;
      lastMiddleState = currentMiddle;

      // Update SmartDashboard down here...
      SmartDashboard.putNumber("Cargo Count", cargoCount);
  }
  /*@Override
  public void periodic() {
// ==========================================
      //           HARDWARE ABSTRACTION LAYER
      // ==========================================
      // Our optical beam-break sensors return FALSE when a ball blocks the light.
      // By adding the exclamation point (!), we invert the logic so our variables 
      // are TRUE when a ball is physically present.
      
      // currentEntry: TRUE when a ball is at the bottom agitator.
      boolean currentEntry = !entrySensor.get(); 
      
      // currentMiddle: TRUE when a ball is halfway up the indexer.
      boolean currentMiddle = !middleSensor.get(); 
      
      // currentExit: TRUE when a ball is touching the top shooter plates.
      boolean currentExit = !exitSensor.get();

      boolean deploySensor = !intakeDeploySensor.get();
    // --- ENTRANCE LOGIC ---
    // Grab the true physical direction of the cargo from the Agitator
    double agitatorSpeed = frc.robot.RobotContainer.agitator.getSpeed();
    double indexerSpeed = frc.robot.RobotContainer.indexer.getSpeed();

    // RISING EDGE: Switch Lifts (Cargo entering the Agitator)
    if (currentEntry && !trigger && debounceTimer.get() > 0.25 && deploySensor) {
        // ONLY count up if the agitator is intentionally driving INWARD
        if (agitatorSpeed > 0.1) { 
              cargoCount++;   // Increment cargo count
              trigger = true;    // Locks your counter
            isStaging = true;  // Wakes up the auto-index motor!
        }
        debounceTimer.reset();
    }

    // FALLING EDGE: Switch Presses (Cargo exiting the Agitator)
    if (!currentEntry && lastEntryState && debounceTimer.get() > 0.25) {
        // ONLY count down if the agitator is intentionally driving OUTWARD (Ejecting)
        if (agitatorSpeed < -0.1) { 
              if(cargoCount > 0){
                 cargoCount--;
                 trigger = false;
              }
           
            
        }
        debounceTimer.reset();
    }

    // --- EXIT LOGIC ---
    if (!currentExit && lastExitState && debounceTimer.get() > 0.25) {
        cargoCount--;
        debounceTimer.reset();
    }

    // --- CLAMPING ---
    if (cargoCount < 0) {
        cargoCount = 0;
    }
    // Hard-cap at 3 to fix the LED Matrix Bug!
    if (cargoCount > 3) {
        cargoCount = 3; 
    }
    // auto indexes the first cargo past middle sensor
  if(trigger && agitatorSpeed > 0.1){
 if (!middleSensor.get()){
  trigger = false;
 }
 if(cargoCount == 2 && !currentEntry && lastEntryState && debounceTimer.get() > 0.25){
trigger = false;
debounceTimer.reset();
    }}


    // The Never-Sleeping Auto-Index Watcher
    if (isStaging) {
        // Falling Edge: Ball was here last loop, but is clear this loop!
        if (!currentMiddle && lastMiddleState) {
            isStaging = false; // Turn off the flag!
        }
    }
// THE HARD CEILING FAILSAFE: 
      // Stop staging if it hits the top sensor OR if it physically bumps the flywheel!
      if (currentExit || RobotContainer.shooter.isBeingBackdriven()) {
          isStaging = false;
      }
    // Save states for the next loop
    lastEntryState = currentEntry;
    lastExitState = currentExit;
    lastMiddleState = currentMiddle;

    // Output to Driver Station
    SmartDashboard.putNumber("Cargo Count", cargoCount);
    SmartDashboard.putBoolean("Middle Sensor Blocked", !middleSensor.get());
      SmartDashboard.putBoolean("Exit Sensor Blocked", !exitSensor.get());
      SmartDashboard.putBoolean("Entry Sensor lifted", entrySensor.get());
            SmartDashboard.putBoolean("Deploy Sensor", !intakeDeploySensor.get());
                SmartDashboard.putBoolean("trigger", trigger);
                    SmartDashboard.putBoolean("is staging", isStaging);
  }/* */
}