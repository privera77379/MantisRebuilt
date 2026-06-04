package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(8);
  //might swap for the PH since we dont use pnumatics anymore, so minus well use the tech if we have it.
  private final Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
  //if we end up using eht REV PH we can just change the PneumaticsModuleType and it should work since the solenoid is on the same port, but if we end up using a different solenoid or port we can just change it here without having to change any code in the commands that use the intake.
  //private final Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.REVPH, 7);
  public Intake() {
    intakeMotor.setInverted(true);
  }
private boolean isIntaking = false;
// Control methods for the intake motor
  public void setSpeed(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    isIntaking = speed > 0.1; // True if running forward
  }
// Convenience method to stop the intake
  public void stop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    isIntaking = false;
  }

public boolean isIntaking() {
    // TalonSRX uses Phoenix 5 API
    // Get the current motor output percentage (between -1.0 and 1.0)
    double speed = intakeMotor.getMotorOutputPercent();
    return (Math.abs(speed) > 0.1);
  }
// Control methods for the solenoid
  public void deploy() {
    deploySolenoid.set(true);
  }
// Retract the intake by setting the solenoid to false
  public void retract() {
    deploySolenoid.set(false);
  }
}