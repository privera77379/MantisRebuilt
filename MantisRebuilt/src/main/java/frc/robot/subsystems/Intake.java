package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(8);
  private final Timer retractTimer = new Timer();
  private boolean commLossLockout = false;
//placed the pneumatics here since this was the first logical place they would be used,
  private final Solenoid deploySolenoid = new Solenoid(5, PneumaticsModuleType.REVPH, 0);
  private final Compressor compressor = new Compressor(5, PneumaticsModuleType.REVPH);

  // Intake Deployment limit switch 
  private final DigitalInput intakeDeploySensor = new DigitalInput(3); 


//sets up the intake of the robot and the compressor and lockout timer for flickering disconnects or brownouts
  public Intake() {
    intakeMotor.setInverted(true);
    compressor.enableDigital();
    retractTimer.start(); 
    resetLockout();
  }
//sets intake speed
  public void setSpeed(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }
//stops intake
  public void stop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }
//coms loss timer reset to stop the pneumatics from flickering back and forth on disconnects
  public void resetLockout() {
      commLossLockout = false;
  }
    //intaking check for the cargo counter 
  public boolean isIntaking() {
    double speed = intakeMotor.getMotorOutputPercent();
    return (Math.abs(speed) > 0.05);
  }
//deployed getter to show the intake sensor has been tripped and thus is capable of intaking a cargo now (helps with erroneous cargo counts when the deployment shakes the agitator and thus the limit switch attached to it)
  public boolean isDeployed() {
      return !intakeDeploySensor.get();
  }
//deploy pneumatics method, not on coms lock out condition
  public void deploy() {
    if (!commLossLockout) {
        deploySolenoid.set(true);
    }
  }
//pulls intake back in.
  public void retract() {
    deploySolenoid.set(false);
  }

  @Override
  public void periodic() {
      // 1. Force compressor OFF during Demo Mode or System Check
      if (RobotContainer.isDemoMode || SmartDashboard.getBoolean("Run System Check", true)) {
          compressor.disable();
      } else {
          compressor.enableDigital();
      }

      if (!DriverStation.isEnabled()) {
          commLossLockout = true;
      }

      // 2. Smart-Deploy Logic
      if (commLossLockout) {
          retract(); 
      } else if (isIntaking()) {
          deploy();
          retractTimer.reset(); 
      } else {
          if (retractTimer.get() > 0.25) {
              retract();
          }
      }

            SmartDashboard.putBoolean("Deploy Sensor", !intakeDeploySensor.get());
  }
}