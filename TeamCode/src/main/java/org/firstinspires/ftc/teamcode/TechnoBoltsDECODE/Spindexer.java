package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindexer {

    // =====================================
    // HARDWARE
    // =====================================

    private final Servo spindexerServo;
    private final RevColorSensorV3 colorSensor;

    // =====================================
    // SERVO POSITIONS
    // =====================================

    private final double[] SLOT_POSITIONS = {
            0.10, // Slot 0
            0.45, // Slot 1
            0.80  // Slot 2
    };

    // =====================================
    // STATE
    // =====================================

    private int currentSlot = 0;

    private boolean artifactPreviouslyDetected = false;

    // stability tracking
    private ArtifactColor stableDetection = ArtifactColor.EMPTY;
    private int stableCount = 0;
    private final int REQUIRED_FRAMES = 3;

    // =====================================
    // COLORS
    // =====================================

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        EMPTY
    }

    private final ArtifactColor[] storedArtifacts = {
            ArtifactColor.EMPTY,
            ArtifactColor.EMPTY,
            ArtifactColor.EMPTY
    };

    // =====================================
    // CONSTRUCTOR
    // =====================================

    public Spindexer(HardwareMap hardwareMap) {

        spindexerServo =
                hardwareMap.get(Servo.class, "spindexerServo");

        colorSensor =
                hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        moveToSlot(0);
    }

    // =====================================
    // MOVE TO SLOT
    // =====================================

    public void moveToSlot(int slot) {

        if (slot < 0 || slot > 2) return;

        currentSlot = slot;

        spindexerServo.setPosition(SLOT_POSITIONS[slot]);
    }

    // =====================================
    // NEXT SLOT
    // =====================================

    public void nextSlot() {

        currentSlot++;

        if (currentSlot > 2) {
            currentSlot = 0;
        }

        moveToSlot(currentSlot);
    }

    // =====================================
    // COLOR DETECTION (FIXED)
    // =====================================

    public ArtifactColor detectColor() {

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int total = r + g + b;

        // no signal
        if (total < 50) {
            return ArtifactColor.EMPTY;
        }

        double rn = (double) r / total;
        double gn = (double) g / total;
        double bn = (double) b / total;

        // GREEN (dominant green)
        if (gn > rn + 0.15 && gn > bn + 0.15) {
            return ArtifactColor.GREEN;
        }

        // PURPLE (red + blue dominant, green low)
        if (rn > gn + 0.10 && bn > gn + 0.10) {
            return ArtifactColor.PURPLE;
        }

        return ArtifactColor.EMPTY;
    }

    // =====================================
    // MAIN UPDATE LOOP
    // =====================================

    public void update() {

        ArtifactColor detected = detectColor();

        // stability logic
        if (detected == stableDetection && detected != ArtifactColor.EMPTY) {
            stableCount++;
        } else {
            stableDetection = detected;
            stableCount = 1;
        }

        boolean artifactConfirmed = stableCount >= REQUIRED_FRAMES;

        // rising edge trigger
        if (artifactConfirmed && !artifactPreviouslyDetected) {

            storedArtifacts[currentSlot] = detected;

            nextSlot();
        }

        artifactPreviouslyDetected = artifactConfirmed;
    }

    // =====================================
    // GETTERS
    // =====================================

    public int getCurrentSlot() {
        return currentSlot;
    }

    public ArtifactColor getDetectedColor() {
        return detectColor();
    }

    public String getStoredArtifacts() {
        return "[" +
                storedArtifacts[0] + ", " +
                storedArtifacts[1] + ", " +
                storedArtifacts[2] + "]";
    }

    // =====================================
    // DEBUG
    // =====================================

    public int getRed() {
        return colorSensor.red();
    }

    public int getGreen() {
        return colorSensor.green();
    }

    public int getBlue() {
        return colorSensor.blue();
    }

    public String debugColor() {
        return "R:" + getRed() +
                " G:" + getGreen() +
                " B:" + getBlue();
    }
}