package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    // 1. Hardware Initialization
    private final TalonFX rightFront = new TalonFX(10);
    private final TalonFX rightBack = new TalonFX(1);
    private final TalonFX leftFront = new TalonFX(2);
    private final TalonFX leftBack = new TalonFX(3);

    // The NavX Gyro plugged into the roboRIO SPI port
private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // 2. Control Objects
    // This PID controller calculates how fast to turn to reach the target angle.
    // (P: 0.015, I: 0, D: 0.001) are starter values. You may need to tune these!
    private final PIDController turnController = new PIDController(0.015, 0, 0.001);

    // Phoenix 6 requires request objects to send commands to motors
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

    public Drive() {
        // --- Motor Configuration ---
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        // In Tank Drive, one side of the gearbox is usually reversed
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftFront.getConfigurator().apply(leftConfig);
        leftBack.getConfigurator().apply(leftConfig);
        rightFront.getConfigurator().apply(rightConfig);
        rightBack.getConfigurator().apply(rightConfig);

        // Tell the back motors to mirror exactly what the front motors do
leftBack.setControl(new Follower(leftFront.getDeviceID(), MotorAlignmentValue.Aligned));
rightBack.setControl(new Follower(rightFront.getDeviceID(), MotorAlignmentValue.Aligned));

        // --- PID Configuration ---
        // Tells the math that -180 degrees and 180 degrees are the exact same spot on the circle
        turnController.enableContinuousInput(-180.0, 180.0);
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
        // 1. Where are we facing right now?
        double currentAngle = navx.getYaw();

        // 2. Let the PID math figure out how hard we need to turn
        double turnPower = turnController.calculate(currentAngle, targetAngleDegrees);

        // 3. Put a speed limit on the turn so the robot doesn't whip around violently
        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

        // 4. Feed the throttle and the new calculated turn power into the standard drive code
        arcadeDrive(throttle, turnPower);
    }

    // --- UTILITIES ---
    public double getYaw() {
        return navx.getYaw();
    }

    public void zeroHeading() {
        navx.reset();
    }

    public void stop() {
        leftFront.setControl(leftOut.withOutput(0));
        rightFront.setControl(rightOut.withOutput(0));
    }

    @Override
    public void periodic() {
        // Intentionally blank. No polling controllers in subsystems!
    }
}