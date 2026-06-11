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

    // --- STATE TRACKING ---
    @SuppressWarnings("unused")
    private int lastCargoCount = -1; 
    private double streakPosition = 0.0;

    // --- BRIGHTNESS CONTROLS (0.0 to 1.0) ---
    private final double MATRIX_BRIGHTNESS = 0.20; 

    // --- HARDWARE ZONES ---
    private final int MATRIX_END = 256; 
    private final int STRIPS_END = 271; 
    private final int TOTAL_LENGTH = 288; 

    // ==========================================
    //            LED PIXEL ARRAYS
    // ==========================================
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

    private final int[] num0LEDs = {
         69, 70, 71, 72, 73, 74, 75, 83, 84, 85, 86, 87, 88, 89, 90, 91, 99, 100, 107, 108, 114, 115, 124, 125, 130, 131, 140, 141, 146, 147, 156, 157,163, 164, 171, 172, 180, 181, 182, 183, 184, 185, 186, 187, 197, 198, 199, 200, 201, 202
    };

    
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
    }

    @Override
    public void periodic() {
        boolean isDisabled = DriverStation.isDisabled();
        int currentCargo = indexer.getCargoCount();

        // --- ZONE 1: MATRIX (Updates constantly so it never drops out) ---
        if (isDisabled) {
            drawMatrixLogo(); 
        } else {
            drawMatrixNumber(currentCargo); 
        }

        // --- ZONES 2 & 3: STRIPS & UNDERGLOW ---
        if (isDisabled) {
           // setStripsColor(Color.kBlue);
           runVorTXStreakStrips();
            runUnderglowBreathe();
            m_led.setData(m_buffer);
            return; 
        }

        // --- ENABLED PRIORITY OVERLAYS ---
        // PRIORITY 1: PENALTY WARNING (3+ CARGO) 
        if (currentCargo >= 3) {
            Color penaltyColor = ((System.currentTimeMillis() / 250) % 2 == 0) ? Color.kRed : Color.kYellow;
            setDynamicZonesColor(penaltyColor);
            m_led.setData(m_buffer);
            return; 
        }

        // PRIORITY 2: SHOOTING 
        double shootSpeed = shooter.getShooterSpeed();
        if (shootSpeed > 0.2) {
            Color shootColor = (shootSpeed > 0.6) ? Color.kRed : Color.kOrange;
            Color flashColor = ((System.currentTimeMillis() / 100) % 2 == 0) ? shootColor : Color.kBlack;
            setDynamicZonesColor(flashColor);
            m_led.setData(m_buffer);
            return; 
        }

        // PRIORITY 3: INTAKING 
        if (intake.isIntaking()) {
            Color pulseColor = ((System.currentTimeMillis() / 150) % 2 == 0) ? Color.kWhite : Color.kBlack;
            setDynamicZonesColor(pulseColor);
            m_led.setData(m_buffer);
            return;
        }

        // PRIORITY 4: DEFAULT DRIVING 
        runVorTXStreakStrips();   
         runUnderglowBreathe();       
       // setUnderglowColor(Color.kGreen); 
        
        m_led.setData(m_buffer);
    }

    // ==========================================
    //            ZONE HELPER METHODS
    // ==========================================
    private Color getDimmedColor(Color originalColor, double brightness) {
        return new Color(
            originalColor.red * brightness, 
            originalColor.green * brightness, 
            originalColor.blue * brightness
        );
    }

    private void setDynamicZonesColor(Color color) {
        for (int i = MATRIX_END; i < TOTAL_LENGTH; i++) m_buffer.setLED(i, color);
    }

    @SuppressWarnings("unused")
    private void setStripsColor(Color color) {
        for (int i = MATRIX_END; i < STRIPS_END; i++) m_buffer.setLED(i, color);
    }

    private void setUnderglowColor(Color color) {
        for (int i = STRIPS_END; i < TOTAL_LENGTH; i++) m_buffer.setLED(i, color);
    }

    private void runUnderglowBreathe() {
        double time = Timer.getFPGATimestamp();
        double intensity = (Math.sin(time * 3.0) + 1.0) / 2.0; 
        Color breatheColor = new Color(0.0, intensity, 0.0);
        setUnderglowColor(breatheColor);
    }

    private void runVorTXStreakStrips() {
        int stripLength = STRIPS_END - MATRIX_END; 
        for (int i = MATRIX_END; i < STRIPS_END; i++) {
            int relativeIndex = i - MATRIX_END; 
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

    // --- MATRIX METHODS ---
    private void drawMatrixLogo() {
        for (int i = 0; i < MATRIX_END; i++) m_buffer.setLED(i, getDimmedColor(Color.kBlue, MATRIX_BRIGHTNESS));
        for (int led : logoGreenLEDs) if (led < MATRIX_END) m_buffer.setLED(led, getDimmedColor(Color.kGreen, MATRIX_BRIGHTNESS));
        for (int led : logoWhiteLEDs) if (led < MATRIX_END) m_buffer.setLED(led, getDimmedColor(Color.kWhite, MATRIX_BRIGHTNESS));
    }

    private void drawMatrixNumber(int number) {
        for (int i = 0; i < MATRIX_END; i++) m_buffer.setLED(i, Color.kBlack);
        int[] activeArray = num0LEDs;
        Color numberColor = Color.kWhite;

        if (number == 1) { activeArray = num1LEDs; numberColor = Color.kGreen; } 
        else if (number == 2) { activeArray = num2LEDs; numberColor = Color.kBlue; } 
        else if (number == 3) { activeArray = num3LEDs; numberColor = Color.kRed; }

        for (int led : activeArray) if (led < MATRIX_END) m_buffer.setLED(led, getDimmedColor(numberColor, MATRIX_BRIGHTNESS));
    }
}