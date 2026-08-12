package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// --- CLIMBER SUBSYSTEM ---
// This subsystem is responsible for controlling the climbing mechanism of the robot, which includes two motors and a solenoid for deploying the climbing arms.
// The motors are configured to limit peak current to prevent damage, and the solenoid can be toggled to deploy or retract the arms.
public class Climber extends SubsystemBase {
  private final TalonSRX leftClimber = new TalonSRX(15); 
  private final TalonSRX rightClimber = new TalonSRX(17); 
  private final Solenoid deploySolenoid = new Solenoid(5, PneumaticsModuleType.REVPH, 13);

  public Climber() {
    leftClimber.configPeakCurrentLimit(40);
    rightClimber.configPeakCurrentLimit(40);
    rightClimber.setInverted(true);
  }

  //sets speeds for the climbers simultaneously ( we could also do the follower motor configuration like in the drivetrain, but this saves us from having to asign and reassign the motor configs, since its taxing on the rio's processing)
  public void setBoth(double speed) {
    leftClimber.set(TalonSRXControlMode.PercentOutput, speed);
    rightClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }

  //set just the left motor to rais it up individually (usually to make them both even)
  public void setLeft(double speed) {
    leftClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }
//same but with the right side, the rope being tied at the lengths that they are have slight variations making thier rates occasionally become noticably out of sync
  public void setRight(double speed) {
    rightClimber.set(TalonSRXControlMode.PercentOutput, speed);
  }
//stops the movement of the climbers, how we generally want to end any call to move them, or the motors would move forever
  public void stop() {
    leftClimber.set(TalonSRXControlMode.PercentOutput, 0);
    rightClimber.set(TalonSRXControlMode.PercentOutput, 0);
  }
//sets a toggle to turn off and on the solenoid that pushes the second climber arms out and in
  public void toggleArms() {
    deploySolenoid.toggle();
  }
}