package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.Orchestra;

// We use "public class Drive" so other files (like RobotContainer) can see and use this subsystem. 
// "extends SubsystemBase" registers this file with WPILib's Command Scheduler.
public class Drive extends SubsystemBase {

    // ==========================================
    // 1. HARDWARE INITIALIZATION (Encapsulation)
    // ==========================================
    // We mark motors as "private final" so no other file can directly touch them.
    // All motor changes MUST go through the public methods below!
    private final TalonFX rightFront = new TalonFX(10);
    private final TalonFX rightBack = new TalonFX(1);
    private final TalonFX leftFront = new TalonFX(2);
    private final TalonFX leftBack = new TalonFX(3);

    // The NavX Gyro acts as the robot's compass/gyroscope over SPI
    public final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // ==========================================
    // 2. CONTROL OBJECTS
    // ==========================================
    // PID Controller calculates turning speed to snap to target angles accurately
    private final PIDController turnController = new PIDController(0.006, 0, 0.000);

    // Phoenix 6 requires Request Objects to command speeds to TalonFX motors
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

    // Request Objects for independent rear-motor testing during System Check
    private final DutyCycleOut leftBackOut = new DutyCycleOut(0);
    private final DutyCycleOut rightBackOut = new DutyCycleOut(0);

    // Orchestra uses motor coils as speakers to play audio chirps
    private final static Orchestra falconAudio = new Orchestra();
    
    public Drive() {
        // --- MOTOR CONFIGURATION ---
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        // Invert one side so sending positive power drives both sides FORWARD
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        leftFront.getConfigurator().apply(leftConfig);
        leftBack.getConfigurator().apply(leftConfig);
        rightFront.getConfigurator().apply(rightConfig);
        rightBack.getConfigurator().apply(rightConfig);

        // Tell back motors to mirror front motor signals automatically
        leftBack.setControl(new Follower(leftFront.getDeviceID(), MotorAlignmentValue.Aligned));
        rightBack.setControl(new Follower(rightFront.getDeviceID(), MotorAlignmentValue.Aligned));

        // --- PID SETUP ---
        // Enable continuous input so turning from 180° to -180° takes the shortest path
        turnController.enableContinuousInput(-180, 180);
        turnController.setTolerance(5.0); // Stop turning within 5 degrees

        // --- AUDIO INSTRUMENTS ---
        falconAudio.addInstrument(leftFront);
        falconAudio.addInstrument(rightFront);
        falconAudio.addInstrument(leftBack);
        falconAudio.addInstrument(rightBack);

        restoreStandardDriving();
    }

    // ==========================================
    //   HOW METHODS WORK: "ARCADE DRIVE" EXAMPLE
    // ==========================================
    // A "Method" is a reusable function. Notice the parameters: (double throttle, double steering).
    // "double" means a number with decimals.
    //
    // THE GOLDEN RULE OF SUBSYSTEMS:
    // There is NO Xbox Controller in this file! Subsystems only know how to run 
    // physical hardware given placeholder numbers.
    //
    // 🔗 TREASURE MAP: To see where throttle and steering come from, go to 
    // RobotContainer.java -> drive.setDefaultCommand()
    public void arcadeDrive(double throttle, double steering) {
        // 1. Blend forward/backward power with turning power
        double leftSpeed = throttle + steering;
        double rightSpeed = throttle - steering;

        // 2. Safety Clamp: Scale down both sides if total exceeds 100% (1.0)
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        // 3. Send final values to the master motors
        leftFront.setControl(leftOut.withOutput(leftSpeed));
        rightFront.setControl(rightOut.withOutput(rightSpeed));
    }
// the drive method that turns our robot into tank drive, making the sticks control each side of the robot seperatly
    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftFront.setControl(leftOut.withOutput(leftSpeed));
        rightFront.setControl(rightOut.withOutput(rightSpeed));
    }
// this method is for us to use a point robot to a heading (feild oriented) for accurate shots or passes
    public void snapToAngleDrive(double throttle, double targetAngleDegrees) {
        double currentAngle = navx.getYaw();
        double turnPower = turnController.calculate(currentAngle, targetAngleDegrees);

       // this makes the oversteer problem go away and give it some wiggle room to not be stuck trying to hit an exact .000000x of a distance when using the method, Kill power if inside tolerance zone vs a set point, this prevents the robot from constantly trying to hit it exactly if its close enough
        if (turnController.atSetpoint()) {
            turnPower = 0;
        }

        // Clamp turning speed to 25% max for safe indoor/demo control
        turnPower = Math.max(-0.25, Math.min(0.25, turnPower));
// gives our dashboard info on the snap to angle function, this was for debugging but can be useful info if the robot is too far away to get a good view of and make sure its facing a good angle to drive safely
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX Current Angle", currentAngle);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX Target Angle", targetAngleDegrees);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX Turn Power", turnPower);

        arcadeDrive(throttle, turnPower);
    }

    // ==========================================
    //        GEARBOX DIAGNOSTIC METHODS
    // ==========================================
    
    // These mothods are used during system check to disable one motor on each side, set them to coast and run one set of motors at a time to see if the linkages and motors are good
    
    public void testFrontDrivesBack(double speed) {
        leftBack.setNeutralMode(NeutralModeValue.Coast);
        rightBack.setNeutralMode(NeutralModeValue.Coast);
        leftBack.setControl(new NeutralOut());
        rightBack.setControl(new NeutralOut());
        
        leftFront.setControl(leftOut.withOutput(speed));
        rightFront.setControl(rightOut.withOutput(speed));
    }
    
    public void testBackDrivesFront(double speed) {
        leftFront.setNeutralMode(NeutralModeValue.Coast);
        rightFront.setNeutralMode(NeutralModeValue.Coast);
        leftFront.setControl(new NeutralOut());
        rightFront.setControl(new NeutralOut());
        
        leftBack.setControl(leftBackOut.withOutput(speed));
        rightBack.setControl(rightBackOut.withOutput(speed));
    }
    //this is how we actually check the motor if its spinning during the check, if we have a bad motor or a missing gear or a jam, the motor that is unpowered will not spin and return no velocity, thus meaning we have an issue.
    public boolean isLeftBackSpinning() {
        return Math.abs(leftBack.getVelocity().getValueAsDouble()) > 0.5;
    }
    
    public boolean isRightBackSpinning() {
        return Math.abs(rightBack.getVelocity().getValueAsDouble()) > 0.5;
    }
    
    public boolean isLeftFrontSpinning() {
        return Math.abs(leftFront.getVelocity().getValueAsDouble()) > 0.5;
    }
    
    public boolean isRightFrontSpinning() {
        return Math.abs(rightFront.getVelocity().getValueAsDouble()) > 0.5;
    }

    public void restoreStandardDriving() {
        // Force all drive motors into Brake Mode for crisp stopping & vision tracking
        leftFront.setNeutralMode(NeutralModeValue.Brake);
        leftBack.setNeutralMode(NeutralModeValue.Brake);
        rightFront.setNeutralMode(NeutralModeValue.Brake);
        rightBack.setNeutralMode(NeutralModeValue.Brake);

        // Re-establish master/follower structure
        leftBack.setControl(new Follower(leftFront.getDeviceID(), MotorAlignmentValue.Aligned));
        rightBack.setControl(new Follower(rightFront.getDeviceID(), MotorAlignmentValue.Aligned));
        
        stop(); 
    }
// get yaw is to tell the robot what its facing for the snap to angle drive method
    public double getYaw() {
        return navx.getYaw();
    }
// allows us to tell the robot which way is forward
    public void zeroHeading() {
        navx.reset();
    }
// stops the motors
    public void stop() {
        leftFront.setControl(leftOut.withOutput(0));
        rightFront.setControl(rightOut.withOutput(0));
    }
// limelight autoaim method and tuning
    public void autoAim(Limelight limelight, double forwardSpeed) {
        double kP = 0.0075; // this is the proportional constant for the PID controller, it determines how aggressively the robot will turn to correct its aim based on the error (tx) from the limelight
        double minPower = 0.01; // minimum turn power if its off to ensure we dont motor stall if we are really close but still not exactly on target
        double tolerance = 1.5;// this is how we fix the jittering and oscillation of the robot since it wants to be exactly at the our set point
        double turnPower = 0.0;// no turn by default, incase we are spot on in the manual aim, but we also dont want a value here or we will start turning at the start of every autoaim, maybe even the wrong way
// has target checks if the limelight sees the set target, in this case any april tags, but we can specify that in Limelight.java
        if (limelight.hasTarget()) { //if we have target we do the following
            limelight.setLEDOff(); //if the LED is on, we want it off, its blinding and not that helpful when looking at AprilTags
            double tx = limelight.getTx();//looks for the Limelights aim compared to the target for adjusment
            
            if (Math.abs(tx) < tolerance) { //if we are further away from target then our tolerance we proceed to turn via the ELSE, the abs is because we could be off in either direction, so this eliminates a whole + or - situation
                turnPower = 0.0; //what happens if we are within our tolerance
            } else { //if we are not inside the tolerance
                turnPower = tx * kP;// we multiply the error by the proportional constant to get our turn power, this is a basic P controller, we could add I and D but for this application its not needed
                if (tx > 0) { turnPower += minPower; } //if we are off to the right, we add a minimum power to ensure we turn enough to get on target
                else { turnPower -= minPower; } //if we are off to the left, we subtract a minimum power to ensure we turn enough to get on target
            }
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower)); //clamp the turn power to a maximum of 30% to prevent overshooting and oscillation
        } else {
            limelight.setLEDBlink(); // funny enough, this is what happens if we didnt see the target at all, it just blinks, but sometimes this blink will allow it to catch a glimpse of the target and then not see it again when the light turns off, im not sure that we want this to remain, in reality we could make the LED's flash instead that way we can avoid blinding anyone by pointing the limelight at them and pressing either autoaim
        }

        arcadeDrive(forwardSpeed, turnPower);// using the above turn variable, we loop this function changing the turning of the robot using our already perfectly working arcade drive method. we set our drive speed to 0 usually when we call it, because we dont want the robot moving when we call it, just aiming, but if you wanted to get fancy with it, we could made the function account for distance and incoperate a drive function to put the robot in its "sweet spot zone" to make every shot in autoaim have the highest shot percentage and cargo stay in the goal more consistantly instead of bouncing out
    }
//the mehod for how playing instruments works
    public static void playSong(String filename) {
        falconAudio.loadMusic(filename);
        falconAudio.play();
    }
    //the method that ends the audio playback 
    public static void stopMusic() {
        falconAudio.stop();
    }

@Override // This sticky-note tells the compiler we are replacing a default WPILib method. If we misspell "periodic", it will throw an error instead of secretly ignoring us!
    public void periodic() {        
        // This is the robot's heartbeat. Everything inside this block runs continuously, 
        // 50 times a second, forever. Use this for things that need constant checking 
        // (like reading sensors, updating dashboard numbers, or triggering safety failsafes).
        
        // --- NAVX TELEMETRY ---
        // This will show the current yaw angle of the NavX on the SmartDashboard, which is useful for debugging and tuning our snap-to-angle drive.
       edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("NavX YAW", navx.getYaw());
    }
}