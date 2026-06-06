package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public Indexer() {
    indexTalon.setInverted(false);
    debounceTimer.start();
  }
// Control methods for the indexer motor
  public void setSpeed(double speed) {
    indexTalon.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stop() {
    indexTalon.set(TalonSRXControlMode.PercentOutput, 0);
  }
// Utility method to get the current cargo count, which can be used by commands or displayed on the LED array and tracking system.
  public int getCargoCount() {
    return cargoCount;
  }

  @Override
  public void periodic() {
    // In WPILib, standard beam breaks return FALSE when the beam is broken (object detected)
    //but we will be using a limit switch for the entery sensor and is mounted such that it is pressed by default and released when cargo is present, so it returns TRUE when cargo is detected. The exit sensor is a standard beam break that returns FALSE when cargo is detected. The middle sensor is also a standard beam break that returns FALSE when cargo is detected.
    // We invert it here (!) so true = "Cargo Detected" for easier reading
    boolean currentEntry = entrySensor.get();
    boolean currentExit = !exitSensor.get();

    // --- ENTRANCE LOGIC ---
    
    // RISING EDGE: Cargo just touched the sensor
    if (currentEntry && !lastEntryState) {
        // Only count it if 0.25s have passed (debouncing flutter) 
        // AND the motor is NOT running in reverse
        if (debounceTimer.get() > 0.25 && indexTalon.getMotorOutputPercent() >= -0.1) {
            cargoCount++;
            debounceTimer.reset();
        }
    }

    // FALLING EDGE: Cargo just released the sensor
    if (!currentEntry && lastEntryState) {
        // Only deduct if 0.25s have passed AND the motor is running in reverse (Ejecting)
        if (debounceTimer.get() > 0.25 && indexTalon.getMotorOutputPercent() < -0.1) {
            cargoCount--;
            debounceTimer.reset();
        }
    }

    // --- EXIT LOGIC ---
    // Falling Edge: Cargo cleared the shooter plates (Shot fired!)
    if (!currentExit && lastExitState) {
        if (debounceTimer.get() > 0.25) { // Add a tiny debounce here too just in case the beam flutters!
             cargoCount--;
             debounceTimer.reset();
        }
    }
    // --- CLAMPING (Never go below 0) ---
    // if we start the robot with a preload or cargo inside the robot we dont want firing a cargo to make the count go negative, so we clamp it at 0. This also prevents any weird behavior if a sensor fails and starts giving false readings.
    if (cargoCount < 0) {
        cargoCount = 0;
    }

    // Save states for the next loop
    lastEntryState = currentEntry;
    lastExitState = currentExit;

    // Output to Driver Station
    SmartDashboard.putNumber("Cargo Count", cargoCount);
    SmartDashboard.putBoolean("Middle Sensor Blocked", !middleSensor.get());
      SmartDashboard.putBoolean("Exit Sensor Blocked", !exitSensor.get());
      SmartDashboard.putBoolean("Entry Sensor Blocked", !entrySensor.get());
  }
}