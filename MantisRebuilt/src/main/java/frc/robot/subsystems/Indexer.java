package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
// this subsystem is the middle wheels that transport the cargo from the front agitator up to the shooter
public class Indexer extends SubsystemBase {
  private final TalonSRX indexTalon = new TalonSRX(4);
  private final Timer debounceTimer = new Timer();

  // --- SENSORS ---
  public final DigitalInput entrySensor = new DigitalInput(0); // Agitator sensor (limit switch)
  public final DigitalInput middleSensor = new DigitalInput(1); // Middle verification sensor (beam brake)
  public final DigitalInput exitSensor = new DigitalInput(2); // Shooter plates sensor (beam brake)

  // --- STATE VARIABLES ---
  //we use these variables to count cargo, and track the cargo, but as a means to count it really
  private int cargoCount = 0;
  private boolean lastEntryState = false;
  private boolean lastExitState = false;
  private boolean lastMiddleState = false;
  private boolean isStaging = false;
  private boolean isRetracting = false;
  private final Timer retractTimer = new Timer();
//sets up the indexer motor and timer 
  public Indexer() {
    indexTalon.setInverted(false);
    debounceTimer.start();
  }
// tracks the indexer speed 
  public double getSpeed() {
      return indexTalon.getMotorOutputPercent();
  }
//sets the indexer speed
  public void setSpeed(double speed) {
    indexTalon.set(TalonSRXControlMode.PercentOutput, speed);
  }
// method for other systems to check the cargo count (a getter method if you will, for other methods or systems to "get" this value from this subsystem).
  public int getCargoCount() {
    return cargoCount;
  }
//we want the first cargo to stage itself just past the beam brake, this give the robot room to take in one more and not have them bunched up.
  public void autoIndex(double speed) {
    // 1. Are we in the new Retraction state? do we need to rewind the cargo for any reason
      if (isRetracting) {
          indexTalon.set(TalonSRXControlMode.PercentOutput, -0.2); // Gentle reverse speed
      }
      // Run motor ONLY if holding 1 ball and staging flag is active, our default use 
      else if (getCargoCount() == 1 && isStaging) {//auto index moves the cargo on the first cargo and the is staging check hasnt cleared
          indexTalon.set(TalonSRXControlMode.PercentOutput, speed);
      } 
      // 3. Neither? Stop.
      else {
          indexTalon.set(TalonSRXControlMode.PercentOutput, 0);
      }
  }
//stop indexer motors
  public void stop() {
    indexTalon.set(TalonSRXControlMode.PercentOutput, 0);
  }
//turns off the is staging status we can use this method in other methods to cancel the staging status for a number of reasons
  public void cancelStaging() {
      isStaging = false;
  }

  @Override
  public void periodic() {
    //50 times a second we will get our sensors states and store them in these variables and agitator speed
      boolean currentEntry = entrySensor.get(); 
      boolean currentMiddle = !middleSensor.get(); 
      boolean currentExit = !exitSensor.get(); 

      double agitatorSpeed = RobotContainer.agitator.getSpeed();

      // --- DIRECTION-AWARE ENTRY LOGIC ---
      if (agitatorSpeed > 0.1) {
        if (currentEntry && !lastEntryState && RobotContainer.intake.isDeployed() && cargoCount <= 0) {
              cargoCount++; 
              isStaging = true; 
              debounceTimer.reset();}
          
         else if (currentEntry && !lastEntryState && debounceTimer.get() > 0.25 && RobotContainer.intake.isDeployed() && cargoCount > 0){
          // INTAKING: Rising Edge counts UP and the intake sensor says the intake is down
               cargoCount++; 
              debounceTimer.reset();
            }
      } else if (agitatorSpeed < -0.1) {
          // OUTTAKING: Falling Edge counts DOWN
          if (!currentEntry && lastEntryState) {
              cargoCount--; 
          }
      }

      // --- HARD CEILING FAILSAFE ---
      if (currentExit || RobotContainer.shooter.isBeingBackdriven()) {// if it detects a cargo is in front of the exit beam brake or the motor is being back driven by the cargo moving too far in the indexer and somehow not tripping the sensor its cancels and stops the indexer
          isStaging = false;
          // Start the pullback sequence!
          isRetracting = true;
          retractTimer.restart(); // Resets to 0.0 and starts counting
      }

      // --- AUTO-INDEX WATCHER ---
      //intended loop to shut off the is staging when it sees that it is staging and the middle sensor goes from tripped to open again.
      if (isStaging) {
          if (!currentMiddle && lastMiddleState) {
              isStaging = false; 
          }
      }
       // --- THE RETRACTION WATCHER ---
      // If we are pulling back, wait exactly 0.8 seconds, then stop!
      if (isRetracting && retractTimer.hasElapsed(0.8)) {
          isRetracting = false; // This automatically makes autoIndex() turn the motor to 0
      }
      // --- EXIT LOGIC (Shooting) ---
      if (!currentExit && lastExitState) {
          cargoCount--;
          debounceTimer.reset();
      }

      // --- CLAMP COUNT ---
      if (cargoCount < 0) cargoCount = 0;
      if (cargoCount > 3) cargoCount = 3; 

      // --- SAVE STATES FOR NEXT LOOP ---
      lastEntryState = currentEntry;
      lastExitState = currentExit;
      lastMiddleState = currentMiddle;

      // Dashboard Telemetry
      SmartDashboard.putNumber("Cargo Count", cargoCount);
      SmartDashboard.putBoolean("Middle Sensor Blocked", currentMiddle);
      SmartDashboard.putBoolean("Exit Sensor Blocked", currentExit);
      SmartDashboard.putBoolean("Entry Sensor down", currentEntry);
      SmartDashboard.putBoolean("is staging", isStaging);
  }
}