package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.Timer;



public class Indexer extends SubsystemBase {
  private final TalonSRX indexTalon = new TalonSRX(4);
private final Timer debounceTimer = new Timer();
  // --- SENSORS ---
  // Update these ports to match your RoboRIO wiring!
  public final DigitalInput entrySensor = new DigitalInput(0); // Agitator sensor
  public final DigitalInput middleSensor = new DigitalInput(1); // Middle verification
  public final DigitalInput exitSensor = new DigitalInput(2); // Shooter plates sensor

  // --- STATE VARIABLES ---
  private int cargoCount = 0;
  private boolean lastEntryState = false;
  private boolean lastExitState = false;
  private boolean lastMiddleState = false;
 private boolean trigger = false; // Your existing counting lockout
  private boolean isStaging = false; // NEW: Controls the auto-index motor

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


  @Override
  public void periodic() {
    // In WPILib, standard beam breaks return FALSE when the beam is broken (object detected)
    //but we will be using a limit switch for the entery sensor and is mounted such that it is pressed by default and released when cargo is present, so it returns TRUE when cargo is detected. The exit sensor is a standard beam break that returns FALSE when cargo is detected. The middle sensor is also a standard beam break that returns FALSE when cargo is detected.
    // We invert it here (!) so true = "Cargo Detected" for easier reading
    boolean currentEntry = entrySensor.get();
    boolean currentExit = !exitSensor.get();
    boolean currentMiddle = !middleSensor.get();

    // --- ENTRANCE LOGIC ---
    // Grab the true physical direction of the cargo from the Agitator
    double agitatorSpeed = frc.robot.RobotContainer.agitator.getSpeed();
    double indexerSpeed = frc.robot.RobotContainer.indexer.getSpeed();

    // RISING EDGE: Switch Lifts (Cargo entering the Agitator)
    if (currentEntry && !lastEntryState && !trigger && debounceTimer.get() > 0.25) {
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
  if(trigger && indexerSpeed > 0.1){
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

    // Save states for the next loop
    lastEntryState = currentEntry;
    lastExitState = currentExit;
    lastMiddleState = currentMiddle;

    // Output to Driver Station
    SmartDashboard.putNumber("Cargo Count", cargoCount);
    SmartDashboard.putBoolean("Middle Sensor Blocked", !middleSensor.get());
      SmartDashboard.putBoolean("Exit Sensor Blocked", !exitSensor.get());
      SmartDashboard.putBoolean("Entry Sensor Blocked", !entrySensor.get());
  }
}