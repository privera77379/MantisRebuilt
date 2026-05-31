package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    
    // Subsystem connections
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;

    // State Tracking
    private int lastCargoCount = 0;
    private final Timer animationTimer = new Timer();
    private boolean playingCaptureAnimation = false;
    private Color captureColor = Color.kBlack;
    private double streakPosition = 0.0;

// --- HARDWARE MAPPING ---
    // 256 (Matrix) + 15 (Upper Strips) + 17 (Underglow) = 288 Total
    private final int TOTAL_LENGTH = 288; 

    public LED(Indexer indexer, Intake intake, Shooter shooter) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;

        // Plugged into PWM Port 0 on the RoboRIO
        m_led = new AddressableLED(0); 
        m_buffer = new AddressableLEDBuffer(TOTAL_LENGTH); 
        
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();

        animationTimer.start();
    }

    @Override
    public void periodic() {
        int currentCargo = indexer.getCargoCount();

        // --- EDGE DETECTION: NEW CARGO ANIMATION TRIGGER ---
        // If count goes up, start the 2-flash animation
        if (currentCargo > lastCargoCount) {
            playingCaptureAnimation = true;
            captureColor = (currentCargo == 1) ? Color.kGreen : Color.kBlue;
            animationTimer.reset();
        }
        lastCargoCount = currentCargo;

        // ==========================================
        //         THE PRIORITY STATE MACHINE
        // ==========================================

        // PRIORITY 1: PENALTY WARNING (3+ CARGO)
        if (currentCargo >= 3) {
            // Flash Red and Yellow every 0.25 seconds
            if ((System.currentTimeMillis() / 250) % 2 == 0) {
                setSolidColor(Color.kRed);
            } else {
                setSolidColor(Color.kYellow);
            }
            return; // Exit early! Overrides all animations below it.
        }

        // PRIORITY 2: SHOOTING
        if (shooter.isShooting()) {
            // Flash Red and Black extremely fast like a strobe
            if ((System.currentTimeMillis() / 100) % 2 == 0) {
                setSolidColor(Color.kRed);
            } else {
                setSolidColor(Color.kBlack);
            }
            return; 
        }

        // PRIORITY 3: CARGO CAPTURE ANIMATION
        if (playingCaptureAnimation) {
            if (animationTimer.get() < 0.6) {
                // Flash on/off every 0.15 seconds
                boolean isOn = ((int)(animationTimer.get() / 0.15) % 2) == 0;
                setSolidColor(isOn ? captureColor : Color.kBlack);
                return;
            } else {
                playingCaptureAnimation = false; // Animation finished
            }
        }

        // PRIORITY 4: INTAKING
        if (intake.isIntaking()) {
            // Pulse white
            if ((System.currentTimeMillis() / 150) % 2 == 0) {
                setSolidColor(Color.kWhite);
            } else {
                setSolidColor(Color.kBlack);
            }
            return;
        }

        // PRIORITY 5: DEFAULT DRIVING (VOR-TX STREAK)
        vorTXStreak();
    }

    // --- HELPER METHODS ---
    private void setSolidColor(Color color) {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setLED(i, color);
        }
        m_led.setData(m_buffer);
    }

    private void vorTXStreak() {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            // Recreates your 50/50 chasing streak mathematically safely
            int shiftedIndex = (i + (int)streakPosition) % m_buffer.getLength();
            if (shiftedIndex < m_buffer.getLength() / 2) {
                m_buffer.setLED(i, Color.kGreen);
            } else {
                m_buffer.setLED(i, Color.kBlue);
            }
        }
        streakPosition += 0.5; // Adjust this to change how fast the streak moves
        if (streakPosition >= m_buffer.getLength()) streakPosition = 0;
        
        m_led.setData(m_buffer);
    }
}