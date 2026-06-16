package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable table;

    public Limelight() {
        // Connect to the Limelight's data stream
        table = NetworkTableInstance.getDefault().getTable("limelight");
        // Force the LEDs off the second the robot turns on!
        setLEDOff(); 
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
        return table.getEntry("tv").getDouble(0.0) == 1.0;
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

    @Override
    public void periodic() {
        // Push the vision data to the Dashboard so you can see it while driving!
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight TX", getTx());
        SmartDashboard.putNumber("Limelight TY", getTy());
    }
}