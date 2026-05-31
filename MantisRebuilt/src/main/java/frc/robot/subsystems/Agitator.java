package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  private final TalonSRX agitatorTalon = new TalonSRX(7);

  public void setSpeed(double speed) {
    agitatorTalon.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stop() {
    agitatorTalon.set(TalonSRXControlMode.PercentOutput, 0);
  }
}