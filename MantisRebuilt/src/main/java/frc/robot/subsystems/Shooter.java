package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // IMPORT THIS!
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Shooter extends SubsystemBase {
    private final TalonFX shooterLeft = new TalonFX(23);
    private final DutyCycleOut request = new DutyCycleOut(0);
private final Timer coastTimer = new Timer();
private double commandedSpeed = 0.0;
    // --- THE MAGIC RPM MAP ---
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

// --- TWO MAGIC RPM MAPS ---
    private final InterpolatingDoubleTreeMap outdoorMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap indoorMap = new InterpolatingDoubleTreeMap();

    public Shooter() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterLeft.getConfigurator().apply(leftConfig);
        coastTimer.start();

        // --- OUTDOOR FUNNEL (TAG 7) CALIBRATION ---
        outdoorMap.put(36.0, 0.35);  // 3 Feet (Baseline)
        outdoorMap.put(139.0, 0.90); // 11 Feet, 9 Inches
        outdoorMap.put(170.0, 1.00); // 14 Feet, 6 Inches (Max Range)

        // --- INDOOR TRASHCAN (TAG 1) CALIBRATION ---
        // You will need to physically test and tune these values!
// --- INDOOR TRASHCAN (TAG 1) CALIBRATION ---
        indoorMap.put(25.0, 0.20);   // Example baseline for close range (adjust as needed)
        indoorMap.put(43.0,0.29);
         indoorMap.put(64.0,0.34);
        indoorMap.put(75.0, 0.37); 
    }
public void setSpeed(double speed) {
        commandedSpeed = speed; 
        shooterLeft.setControl(request.withOutput(speed));
        if (Math.abs(speed) > 0.1) { coastTimer.reset(); }
    }
// --- AUTO-SPOOL METHOD ---
    public void setSpeedForDistance(double distanceInches, double tagID) {
        double calculatedPower;

        if (tagID == 1.0) {
            // 1. Get the math from the Indoor map
            calculatedPower = indoorMap.get(distanceInches);
            
            // 2. THE HARD CAP: Force the power to never exceed 38%
            calculatedPower = Math.min(0.375, calculatedPower); 
            
        } else if (tagID == 2.0) {
            // TAG 2: Gym/Cafeteria Mode (UNCAPPED!)
            // Uses the same map, but allows the motor to spin as fast as it calculates
            calculatedPower = indoorMap.get(distanceInches);
            
        }else if (tagID == 26.0) {
            // Default to the Outdoor Funnel math for Tag 7
            calculatedPower = outdoorMap.get(distanceInches);
        } else{
            calculatedPower = 0.0;
        }

        // Send the final, safe power to the motors
        setSpeed(calculatedPower);
    }

    public void stop() {
        commandedSpeed = 0.0; 
        shooterLeft.setControl(request.withOutput(0));
    }

    public boolean isShooting() {
        return (Math.abs(commandedSpeed) > 0.1);
    }
 // another getter method but this one for shooter speed
  public double getShooterSpeed() {
    // Returns a decimal between 0.0 and 1.0 representing motor power
    return Math.abs(shooterLeft.getDutyCycle().getValue());
  }
public boolean isReadyToFire() {
        // Prevent false positives if the motor is off
        if (!isShooting()) return false; 

        double expectedRPS = commandedSpeed * 100.0; 
        double currentRPS = shooterLeft.getVelocity().getValueAsDouble();

        return currentRPS >= (expectedRPS - 12.0);
    }
    
public boolean isBeingBackdriven() {
        boolean commandedOff = !isShooting();
        boolean coastFinished = coastTimer.hasElapsed(2.0);
        
        
        boolean physicallyMoving = Math.abs(shooterLeft.getVelocity().getValueAsDouble()) > 0.25;

        return commandedOff && coastFinished && physicallyMoving;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Target RPS", commandedSpeed * 100.0);
        SmartDashboard.putNumber("Flywheel Current RPS", shooterLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter Backdriven", isBeingBackdriven());
    }
}