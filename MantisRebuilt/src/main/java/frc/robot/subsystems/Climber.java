package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonSRX leftClimber = new TalonSRX(15); 
  private final TalonSRX rightClimber = new TalonSRX(17); 

  public Climber() {
    leftClimber.configPeakCurrentLimit(40);
    rightClimber.setInverted(false);
    rightClimber.configPeakCurrentLimit(40);
    rightClimber.setInverted(true);
  }

  public void setBoth(double speed) {
    leftClimber.set(TalonSRXControlMode.PercentOutput, speed);
    rightClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setLeft(double speed) {
    leftClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setRight(double speed) {
    rightClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stop() {
    leftClimber.set(TalonSRXControlMode.PercentOutput, 0);
    rightClimber.set(TalonSRXControlMode.PercentOutput, 0);
  }
}