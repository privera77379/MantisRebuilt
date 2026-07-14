package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.Timer;


    

   

public class Intake extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(8);
      // The Software Latch
    private final Timer retractTimer = new Timer();
  private boolean commLossLockout = false;

 


//might swap for the PH since we dont use pnumatics anymore, so minus well use the tech if we have it.
  //private final Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
  //if we end up using eht REV PH we can just change the PneumaticsModuleType and it should work since the solenoid is on the same port, but if we end up using a different solenoid or port we can just change it here without having to change any code in the commands that use the intake.
  private final Solenoid deploySolenoid = new Solenoid(5,PneumaticsModuleType.REVPH, 0);
  private final Compressor compressor = new Compressor(5, PneumaticsModuleType.REVPH);
  //private final Compressor pcmCompressor = new Compressor(5,PneumaticsModuleType.REVPH,1);
  public Intake() {
    intakeMotor.setInverted(true);
    compressor.enableDigital();
        intakeMotor.setInverted(true);
    compressor.enableDigital();
    
    // Start the stopwatch when the robot boots up
    retractTimer.start(); 
  }

  @SuppressWarnings("unused")
 // private boolean isIntaking = false;
// Control methods for the intake motor
  public void setSpeed(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  //  isIntaking = speed > 0.05; // True if running forward
  }
// Convenience method to stop the intake
  public void stop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  //  isIntaking = false;
  }
    // A method to reset the latch when you let go of the button
    public void resetLockout() {
        commLossLockout = false;
    }
    
public boolean isIntaking() {
    // TalonSRX uses Phoenix 5 API
    // Get the current motor output percentage (between -1.0 and 1.0)
    double speed = intakeMotor.getMotorOutputPercent();
    return (Math.abs(speed) > 0.05);
  }
// Control methods for the solenoid
  public void deploy() {
 if (!commLossLockout) {
            deploySolenoid.set(true);
        }
  }
// Retract the intake by setting the solenoid to false
  public void retract() {
    deploySolenoid.set(false);
  }
  @Override
  public void periodic() {
      // 1. The Hard Disconnect Lockout (Kept for safety) 
               // If we are in Demo Mode, force the compressor OFF
      if (RobotContainer.isDemoMode) {
          compressor.disable();
      } else {
          // If NOT in demo mode, enable the digital switch normally
          compressor.enableDigital();
      }
      if (!DriverStation.isEnabled()) {
          commLossLockout = true;
      }

      // 2. The Debounced Smart-Deploy
      if (commLossLockout) {
          retract(); 
      } else if (isIntaking()) {
          // If the motor is running, deploy the arm and hold the timer at 0!
          deploy();
          retractTimer.reset(); 
      } else {
          // The motor is reading 0.0. ONLY retract if it has been off for >0.25s!
          if (retractTimer.get() > 0.25) {
              retract();
          }
      }
  }
}

 
