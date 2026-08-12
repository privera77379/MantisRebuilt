package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // IMPORT THIS!
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer; 

public class Shooter extends SubsystemBase {
    private final TalonFX shooterLeft = new TalonFX(23);
    private final DutyCycleOut request = new DutyCycleOut(0);
private final Timer coastTimer = new Timer();
    // --- THE MAGIC RPM MAP ---
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public Shooter() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterLeft.getConfigurator().apply(leftConfig);
         coastTimer.start();
        // --- LOAD YOUR EMPIRICAL DATA ---
        // Format: rpmMap.put(DistanceInInches, MotorPower);
        rpmMap.put(36.0, 0.35);  // 3 Feet (Baseline)
        rpmMap.put(139.0, 0.90); // 11 Feet, 9 Inches
        rpmMap.put(170.0, 1.00); // 14 Feet, 6 Inches (Max Range)
    }


public void setSpeed(double speed) {
        shooterLeft.setControl(request.withOutput(speed));
        
        // If we are actively shooting, constantly reset the stopwatch to 0
        if (Math.abs(speed) > 0.1) {
            coastTimer.reset(); 
        }
    }
    // --- AUTO-SPOOL METHOD ---
    public void setSpeedForDistance(double distanceInches) {
        // Automatically calculates the perfect power based on your map!
        double calculatedPower = rpmMap.get(distanceInches);
        setSpeed(calculatedPower);
    }

    public void stop() {
        shooterLeft.setControl(request.withOutput(0));
    }


public boolean isShooting() {
    // Get the current duty cycle (output) from the motor signal
    double speed = shooterLeft.getDutyCycle().getValue();
    return (Math.abs(speed) > 0.1);
  }
 // another getter method but this one for shooter speed
  public double getShooterSpeed() {
    // Returns a decimal between 0.0 and 1.0 representing motor power
    return Math.abs(shooterLeft.getDutyCycle().getValue());
  }
public boolean isReadyToFire() {
        double commandedPower = shooterLeft.getDutyCycle().getValueAsDouble();
        if (commandedPower < 0.1) return false; 

        double expectedRPS = commandedPower * 100.0; 
        double currentRPS = shooterLeft.getVelocity().getValueAsDouble();

        // Increased the buffer from 5.0 to 12.0 Rotations Per Second!
        // This gives the wheel room to "breathe" without shutting off the indexer.
        return currentRPS >= (expectedRPS - 12.0);
    }
    
    public boolean isBeingBackdriven() {
        // 1. Are we commanding 0 power?
        boolean commandedOff = !isShooting();
        
        // 2. Has it been at least 2 seconds since we last shot? (Coast period is over)
        boolean coastFinished = coastTimer.hasElapsed(2.0);
        
        // 3. Is the physical shaft spinning?
        boolean physicallyMoving = Math.abs(shooterLeft.getVelocity().getValueAsDouble()) > 0.5;

        // ONLY scream "Backdrive!" if we are off, we are done coasting, and we are moving.
        return commandedOff && coastFinished && physicallyMoving;
    }
}