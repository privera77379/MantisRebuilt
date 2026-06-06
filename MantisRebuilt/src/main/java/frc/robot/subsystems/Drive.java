package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // 1. Hardware Initialization
    private final TalonFX rightFront = new TalonFX(10);
    private final TalonFX rightBack = new TalonFX(1);
    private final TalonFX leftFront = new TalonFX(2);
    private final TalonFX leftBack = new TalonFX(3);

    // The NavX Gyro plugged into the roboRIO SPI port
public final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // 2. Control Objects
    // This PID controller calculates how fast to turn to reach the target angle.
    // (P: 0.015, I: 0, D: 0.001) are starter values. these were too jittery and too quick at turning
     // P = 0.004 (Gas Pedal), I = 0, D = 0.0004 (Brakes) test values for breaks 
    private final PIDController turnController = new PIDController(0.006, 0, 0.000);


    // Phoenix 6 requires request objects to send commands to motors
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

    public Drive() {
        // --- Motor Configurati--
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        // In Tank Drive, one side of the gearbox is usually reversed
             leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
             rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftFront.getConfigurator().apply(leftConfig);
        leftBack.getConfigurator().apply(leftConfig);
        rightFront.getConfigurator().apply(rightConfig);
        rightBack.getConfigurator().apply(rightConfig);

        // Tell the back motors to mirror exactly what the front motors do
             leftBack.setControl(new Follower(leftFront.getDeviceID(), MotorAlignmentValue.Aligned));
             rightBack.setControl(new Follower(rightFront.getDeviceID(), MotorAlignmentValue.Aligned));

// --- PID CONFIGURATION ---
        // Tell the math that -180 degrees and 180 degrees are the exact same point
        // This forces the robot to always take the shortest path to the angle!
        turnController.enableContinuousInput(-180, 180);
        
        //  If we are within 2 degrees, stop trying to turn. I may need to adjust this to get "close enough" without jittering.
        turnController.setTolerance(5.0);
    }

// --- STANDARD ARCADE / RACING DRIVE ---
    public void arcadeDrive(double throttle, double steering) {
        double leftSpeed = throttle + steering;
        double rightSpeed = throttle - steering;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFront.setControl(leftOut.withOutput(leftSpeed));
        rightFront.setControl(rightOut.withOutput(rightSpeed));
    }
    // --- NEW: TANK DRIVE ---
    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftFront.setControl(leftOut.withOutput(leftSpeed));
        rightFront.setControl(rightOut.withOutput(rightSpeed));
    }

    // --- NEW: FIELD ORIENTED SNAP DRIVE ---
public void snapToAngleDrive(double throttle, double targetAngleDegrees) {
        double currentAngle = navx.getYaw();
        
        double turnPower = turnController.calculate(currentAngle, targetAngleDegrees);

        // --- THE JITTER FIX ---
        // If the NavX is within our 2-degree tolerance zone, kill the turning power
        if (turnController.atSetpoint()) {
            turnPower = 0;
        }

        // Clamp the max speed so it doesn't whip around
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));

        // Telemetry
        // This will show the current angle, target angle, and turn power on the SmartDashboard for tuning purposes.
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX Current Angle", currentAngle);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX Target Angle", targetAngleDegrees);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX Turn Power", turnPower);
// Finally, call the standard arcade drive with our throttle and the calculated turn power
        arcadeDrive(throttle, turnPower);
    }

    // --- UTILITIES ---
    // This method gets the Yaw of the Navx, which is going to be used in our Snap to angle drive method, also so we can call it and see it on the shuffleboard later.
    public double getYaw() {
        return navx.getYaw();
    }
// This method resets the NavX's current angle to zero. This is useful for recalibrating the robot's orientation at the start of a match or after a collision.
    public void zeroHeading() {
        navx.reset();
    
    }
// This method immediately stops all drive motors, which can be called in an emergency or at the end of a match to ensure the robot doesn't keep moving.
    public void stop() {
        leftFront.setControl(leftOut.withOutput(0));
        rightFront.setControl(rightOut.withOutput(0));
    }

 @Override
    public void periodic() {
        // --- NAVX TELEMETRY ---
        // This will show the current yaw angle of the NavX on the SmartDashboard, which is useful for debugging and tuning our snap-to-angle drive.
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX YAW", navx.getYaw());
    }
}