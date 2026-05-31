package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(8);
  private final Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

  public Intake() {
    intakeMotor.setInverted(false);
  }
private boolean isIntaking = false;

  public void setSpeed(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    isIntaking = speed > 0.1; // True if running forward
  }

  public void stop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    isIntaking = false;
  }

public boolean isIntaking() {
    // TalonSRX uses Phoenix 5 API
    double speed = intakeMotor.getMotorOutputPercent();
    return (Math.abs(speed) > 0.1);
  }

  public void deploy() {
    deploySolenoid.set(true);
  }

  public void retract() {
    deploySolenoid.set(false);
  }
}