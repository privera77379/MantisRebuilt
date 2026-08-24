package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class Limelight extends SubsystemBase {
    private final NetworkTable table;//sets up a network table for the limelight to use
    private final SendableChooser<String> targetChooser = new SendableChooser<>();
    private final Timer targetTimer = new Timer();

    public Limelight() {
        // Connect to the Limelight's data stream
        table = NetworkTableInstance.getDefault().getTable("limelight");
        // Force the LEDs off the second the robot turns on!
        targetTimer.start(); // Start the stopwatch when the robot boots
        setLEDOff(); 
        // --- SETUP THE TARGET DROPDOWN ---
        targetChooser.setDefaultOption("In-Class (AprilTag @ 51.5in)", "IN_CLASS");
        targetChooser.addOption("Outside Config (Funnel @ 46.0in)", "OUTSIDE");
        SmartDashboard.putData("Target Configuration", targetChooser);
    }

    // --- LED CONTROLS ---
    public void setLEDOff() {
        table.getEntry("ledMode").setNumber(1);
    }

    public void setLEDBlink() {
        table.getEntry("ledMode").setNumber(2);
    }

    public void setLEDOn() {
        table.getEntry("ledMode").setNumber(3);
    }
 // tv: Whether the limelight has any valid targets (0 or 1)
    public boolean hasTarget() {
        boolean seesSomething = table.getEntry("tv").getDouble(0.0) == 1.0;
        
        if (!seesSomething) {
            return false;
        }

        // If it sees a tag, check if it's one of our valid goals!
        double tagID = getTargetID();
        return tagID == 1.0 || tagID == 2.0 || tagID == 26.0; 
    }
    // tid: Returns the ID of the primary AprilTag currently in view. 
    // Returns -1.0 if no tag is seen.
    public double getTargetID() {
        return table.getEntry("tid").getDouble(-1.0);
    }

    // tx: Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    // ty: Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }

    // ta: Target Area (0% of image to 100% of image)
    public double getTa() {
        return table.getEntry("ta").getDouble(0.0);
    }
    // --- DISTANCE CALCULATION ---
// --- SMART DISTANCE CALCULATION ---
public double getDistanceToTarget() {
        if (!hasTarget()) return 0.0;

        // Pull 3D relative position array: [X (Right/Left), Y (Forward/Back), Z (Up/Down)]
        double[] targetPose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        
        if (targetPose.length < 2) return 0.0;

        double sideMeters = targetPose[0];    // Left/Right distance
        double forwardMeters = targetPose[1]; // Straight ahead distance

        // Pythagorean Theorem for ground-plane distance
        double distanceMeters = Math.hypot(forwardMeters, sideMeters);

        // Convert meters to inches (1 meter = 39.37 inches)
        return distanceMeters * 39.37;
    }
    // --- TARGETING CHECKS ---
    // Returns true ONLY if the target is found AND the crosshair is within the tolerance
 // --- THE DEBOUNCED TARGET CHECK --- this is to compensate for the issue of the limelight loosing confirmation on the april tag rapidly flashing, mostly an issue outdoors or at niche distances
    public boolean hasTargetDebounced() {
        if (hasTarget()) {
            targetTimer.reset(); // As long as we see it, keep resetting the clock to 0
            return true;
        }
        // If we lost it, check if it's been missing for less than a quarter second
        return targetTimer.get() < 0.25; 
    }

    // tells us the robot is centered long enough to stay focused even if the robot momentarily loses sight of the target without restarting the aim process
    public boolean isCentered(double toleranceDegrees) {
        if (!hasTargetDebounced()) {
            return false;
        }
        return Math.abs(getTx()) < toleranceDegrees;
    }

@Override
    public void periodic() {
        double currentTag = getTargetID();

        // --- AUTOMATIC PIPELINE SWITCHER ---
        if (currentTag == 1.0 || currentTag == 2.0) {
            // Pipeline 0 has the +11" forward and -24.5" down 3D offsets built-in
            table.getEntry("pipeline").setNumber(0);
        } else if (currentTag == 26.0) {
            // Pipeline 1 has 0 offset for the outdoor funnel
            table.getEntry("pipeline").setNumber(1);
        }

        // Dashboard Telemetry
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight TX", getTx());
        SmartDashboard.putNumber("Limelight TY", getTy());
        SmartDashboard.putNumber("Limelight Tag ID", getTargetID());
        SmartDashboard.putNumber("Distance To Target (Inches)", getDistanceToTarget());
    }
}