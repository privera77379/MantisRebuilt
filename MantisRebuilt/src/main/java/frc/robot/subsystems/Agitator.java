package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// --- AGITATOR SUBSYSTEM ---
// This subsystem is responsible for keeping cargo moving in the indexer and preventing jams.
public class Agitator extends SubsystemBase {
  private final TalonSRX agitatorTalon = new TalonSRX(7);
public Agitator() {
    agitatorTalon.setInverted(false); 
}
public double getSpeed() {
      // Returns actual power from -1.0 to 1.0
      return agitatorTalon.getMotorOutputPercent();
  }
// Simple control methods for the agitator motor
  public void setSpeed(double speed) {
    agitatorTalon.set(TalonSRXControlMode.PercentOutput, speed);
  }
// Convenience method to stop the agitator
  public void stop() {
    agitatorTalon.set(TalonSRXControlMode.PercentOutput, 0);
  }
}