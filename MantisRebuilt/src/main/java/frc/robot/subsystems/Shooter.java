package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterLeft = new TalonFX(23);
  //private final TalonFX shooterRight = new TalonFX(6);
  private final DutyCycleOut request = new DutyCycleOut(0);

  public Shooter() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
   // TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
   // rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterLeft.getConfigurator().apply(leftConfig);
    //shooterRight.getConfigurator().apply(rightConfig);

    // Opposed alignment for Phoenix 6 shooter wheels
    //shooterRight.setControl(new Follower(shooterLeft.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void setSpeed(double speed) {
    shooterLeft.setControl(request.withOutput(speed));
  }

  public void stop() {
    shooterLeft.setControl(request.withOutput(0));
  }
public boolean isShooting() {
    // Get the current duty cycle (output) from the motor signal
    double speed = shooterLeft.getDutyCycle().getValue();
    return (Math.abs(speed) > 0.1);
  }
  // --- ADD THIS METHOD TO SHOOTER.JAVA ---
  public double getShooterSpeed() {
    // Returns a decimal between 0.0 and 1.0 representing motor power
    return Math.abs(shooterLeft.getDutyCycle().getValue());
  }
  
}