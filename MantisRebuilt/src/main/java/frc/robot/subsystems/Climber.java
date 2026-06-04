package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// --- CLIMBER SUBSYSTEM ---
// This subsystem controls the climbing mechanism, which consists of two motors that can be run independently or together. The motors are configured with a current limit to prevent damage during climbing. The right motor is inverted to ensure both sides move in the same direction when given the same speed command.
public class Climber extends SubsystemBase {
  private final TalonSRX leftClimber = new TalonSRX(15); 
  private final TalonSRX rightClimber = new TalonSRX(17); 

  public Climber() {
    leftClimber.configPeakCurrentLimit(40);
    rightClimber.setInverted(false);
    rightClimber.configPeakCurrentLimit(40);
    rightClimber.setInverted(true);
  }
// Control methods for the climber motors
  public void setBoth(double speed) {
    leftClimber.set(TalonSRXControlMode.PercentOutput, speed);
    rightClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }
// Allows independent control of each side of the climber, useful for fine-tuning or correcting alignment during climbing
  public void setLeft(double speed) {
    leftClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }
// Allows independent control of each side of the climber, useful for fine-tuning or correcting alignment during climbing
  public void setRight(double speed) {
    rightClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }
// Convenience method to stop both climber motors immediately
  public void stop() {
    leftClimber.set(TalonSRXControlMode.PercentOutput, 0);
    rightClimber.set(TalonSRXControlMode.PercentOutput, 0);
  }
}