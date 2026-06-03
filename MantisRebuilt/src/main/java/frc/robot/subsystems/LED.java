package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
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
    private boolean showingNumber = false;
    private double streakPosition = 0.0;

    // --- HARDWARE MAPPING ---
    private final int MATRIX_LENGTH = 256; 
    private final int TOTAL_LENGTH = 288; 

    // ==========================================
    //            LED PIXEL ARRAYS
    // ==========================================

    // --- VORTX LOGO ARRAYS (From Excel) ---
    private final int[] logoGreenLEDs = {
        17, 19, 20, 41, 42, 43, 44, 46, 49, 51, 54, 55, 56, 69, 70, 71, 78, 81, 89, 
        90, 91, 99, 100, 110, 113, 124, 125, 142, 145, 154, 155, 156, 164, 165, 166, 
        174, 177, 178, 184, 185, 186, 199, 200, 201, 204, 205, 206, 209, 213, 214, 
        234, 235, 236, 242, 243
    };

    private final int[] logoWhiteLEDs = {
        84, 85, 104, 105, 106, 107, 116, 117, 118, 119, 120, 135, 136, 137, 138, 139, 
        148, 149, 150, 151, 170, 171
    };

    // --- NUMBER ARRAYS (Auto-Calculated for Vertical Serpentine!) ---
    private final int[] num1LEDs = {
        92, 99, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 131, 132, 133, 
        134, 135, 136, 137, 138, 139, 140, 148, 156, 163, 170
    }; 
    
    private final int[] num2LEDs = {
        84, 85, 91, 92, 99, 100, 105, 106, 107, 108, 115, 118, 119, 123, 124, 131, 
        132, 135, 136, 140, 147, 148, 152, 153, 154, 155, 156, 163, 164, 165, 166, 
        170, 171
    }; 
    
    private final int[] num3LEDs = {
        84, 85, 89, 90, 91, 99, 100, 101, 102, 103, 105, 106, 107, 108, 115, 118, 
        119, 120, 124, 131, 136, 140, 147, 148, 155, 156, 164, 171
    }; 


    public LED(Indexer indexer, Intake intake, Shooter shooter) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;

        m_led = new AddressableLED(0); 
        m_buffer = new AddressableLEDBuffer(TOTAL_LENGTH); 
        
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();

        animationTimer.start();
    }

    @Override
    public void periodic() {
        // --- DISABLED STATE ---
        if (DriverStation.isDisabled()) {
            drawVorTXLogo();
            drawStaticStrips();
            m_led.setData(m_buffer);
            return; 
        }

        int currentCargo = indexer.getCargoCount();

        // --- EDGE DETECTION: CARGO COUNT CHANGED ---
        // If count goes UP or DOWN, flash the new number for 1.5 seconds!
        if (currentCargo != lastCargoCount) {
            showingNumber = true;
            animationTimer.reset();
        }
        lastCargoCount = currentCargo;

        // ==========================================
        //         THE PRIORITY STATE MACHINE
        // ==========================================

        // PRIORITY 1: PENALTY WARNING (3+ CARGO) 
        if (currentCargo >= 3) {
            if ((System.currentTimeMillis() / 250) % 2 == 0) {
                setWholeRobotColor(Color.kRed);
            } else {
                setWholeRobotColor(Color.kYellow);
            }
            return; 
        }

        // PRIORITY 2: SHOOTING 
        double shootSpeed = shooter.getShooterSpeed();
        if (shootSpeed > 0.1) {
            // Speed threshold for Red (High) vs Orange (Low)
            Color shootColor = (shootSpeed > 0.6) ? Color.kRed : Color.kOrange;
            if ((System.currentTimeMillis() / 100) % 2 == 0) {
                setWholeRobotColor(shootColor);
            } else {
                setWholeRobotColor(Color.kBlack);
            }
            return; 
        }

        // PRIORITY 3: NUMBER DISPLAY ANIMATION
        if (showingNumber) {
            if (animationTimer.get() < 1.5) { 
                drawNumber(currentCargo);
                runVorTXStreak(); // Keep the strips moving while the number shows
                m_led.setData(m_buffer);
                return;
            } else {
                showingNumber = false; 
            }
        }

        // PRIORITY 4: INTAKING 
        if (intake.isIntaking()) {
            if ((System.currentTimeMillis() / 150) % 2 == 0) {
                setWholeRobotColor(Color.kWhite);
            } else {
                setWholeRobotColor(Color.kBlack);
            }
            return;
        }

        // PRIORITY 5: DEFAULT DRIVING 
        drawVorTXLogo(); 
        runVorTXStreak(); 
        m_led.setData(m_buffer);
    }

    // ==========================================
    //            ZONE HELPER METHODS
    // ==========================================

    private void setWholeRobotColor(Color color) {
        for (int i = 0; i < TOTAL_LENGTH; i++) {
            m_buffer.setLED(i, color);
        }
        m_led.setData(m_buffer);
    }

    private void drawVorTXLogo() {
        // 1. Paint the matrix background Blue
        for (int i = 0; i < MATRIX_LENGTH; i++) {
            m_buffer.setLED(i, Color.kBlue);
        }
        // 2. Paint the Green Logo pixels
        for (int led : logoGreenLEDs) {
            if (led < MATRIX_LENGTH) m_buffer.setLED(led, Color.kGreen);
        }
        // 3. Paint the White Logo pixels
        for (int led : logoWhiteLEDs) {
            if (led < MATRIX_LENGTH) m_buffer.setLED(led, Color.kWhite);
        }
    }

    private void drawNumber(int number) {
        // 1. Paint the matrix background Black so the number pops!
        for (int i = 0; i < MATRIX_LENGTH; i++) {
            m_buffer.setLED(i, Color.kBlack);
        }

        // 2. Pick the correct array based on the cargo count
        int[] activeArray = new int[0];
        Color numberColor = Color.kWhite;

        if (number == 1) {
            activeArray = num1LEDs;
            numberColor = Color.kGreen;
        } else if (number == 2) {
            activeArray = num2LEDs;
            numberColor = Color.kBlue;
        } else if (number == 3) {
            activeArray = num3LEDs;
            numberColor = Color.kRed;
        }

        // 3. Paint the number
        for (int led : activeArray) {
            if (led < MATRIX_LENGTH) m_buffer.setLED(led, numberColor);
        }
    }

    private void drawStaticStrips() {
        for (int i = MATRIX_LENGTH; i < TOTAL_LENGTH; i++) {
            m_buffer.setLED(i, Color.kBlue);
        }
    }

    private void runVorTXStreak() {
        int stripLength = TOTAL_LENGTH - MATRIX_LENGTH; 
        for (int i = MATRIX_LENGTH; i < TOTAL_LENGTH; i++) {
            int relativeIndex = i - MATRIX_LENGTH; 
            int shiftedIndex = (relativeIndex + (int)streakPosition) % stripLength;
            if (shiftedIndex < stripLength / 2) {
                m_buffer.setLED(i, Color.kGreen);
            } else {
                m_buffer.setLED(i, Color.kBlue);
            }
        }
        streakPosition += 0.5; 
        if (streakPosition >= stripLength) streakPosition = 0;
    }
}