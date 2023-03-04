package org.troyargonauts.common.control.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private NetworkTableInstance table = null;
    /**
     * There are 3 states that the Limelight's LEDs can be in:
     *
     * <ul>
     * <li> OFF - Turns off the Limelight's LEDs </li>
     * <li> BLINK - Turns on the Limelight's LEDs and blinks them </li>
     * <li> ON - Turns on the Limelight's LEDs </li>
     * </ul>
     */
    public enum LightMode {
        ON, OFF, BLINK
    }

    /**
     * There are 2 states that the Limelight's Camera can be in:
     *
     * <ul>
     * <li> VISION - Turns on the Limelight's vision processing pipeline </li>
     * <li> DRIVER - Turns on the Limelight's driver mode </li>
     * </ul>
     */
    public enum CameraMode {
        VISION, DRIVER
    }

    /**
     * There should only be one Limelight instance as there should only be one Limelight
     */
    private final static Limelight INSTANCE = new Limelight();

    /**
     * Empty constructor for the Limelight class
     * We should not be creating a new Limelight, thus the constructor is private and empty
     */
    private Limelight() {

    }

    /**
     * Gets the Limelight instance
     * @return the Limelight instance
     */
    public static Limelight getInstance() {
        return INSTANCE;
    }

    /**
     * Horizontal offset from crosshair to target (27 degrees to -27 degrees)
     *
     * <ul>
     * <li> 0 degrees means that the target is centered in the crosshair </li>
     * <li> 27 degrees means that the target is all the way the right of the image. </li>
     * <li> -27 degrees means that the target is all the way to the left of the image. </li>
     *</ul>
     *
     * @return the horizontal offset of the target relative to the crosshair
     */
    public double getX() {
        return getValue("tx").getDouble(0.00);
    }

    /**
     * Vertixal offset from crosshair to target (20.5 degrees to -20.5 degrees)
     *
     * <ul>
     * <li> 0 degrees means that the target is centered in the crosshair </li>
     * <li> 20.5 degrees means that the target is at the top of the image. </li>
     * <li> -20.5 degrees means that the target is at the bottom to the left of the image. </li>
     *</ul>
     *
     * @return the vertical offset of the target relative to the crosshair
     */
    public double getY() {
        return getValue("ty").getDouble(0.00);
    }

    /**
     * Area that the detected target takes up in total Camera iamge (0% to 100%).
     *
     * @return Area of target.
     */
    public double getTa() {
        return getValue("ta").getDouble(0.00);
    }

    /**
     * Gets target skew or rotation (-90 degrees to 90 degrees).
     * The skew is the angle of the target relative to the crosshair.
     *
     * @return Target skew.
     */
    public double getTs() {
        return getValue("ts").getDouble(0.00);
    }

    /**
     * Gets target latency (ms).
     *
     * @return Target latency.
     */
    public double getTl() {
        return getValue("tl").getDouble(0.00);
    }

    /**
     * Sets LED mode of Limelight.
     *
     * @param mode - Light mode for Limelight.
     */
    public void setLedMode(LightMode mode) {
        getValue("ledMode").setNumber(mode.ordinal());
    }

    /**
     * Sets camera mode for Limelight.
     *
     * @param mode - Camera mode for Limelight.
     */
    public void setCameraMode(CameraMode mode) {
        getValue("camMode").setNumber(mode.ordinal());
    }

    /**
     * Sets pipeline number (0-9 value).
     *
     * @param number - Pipeline number (0-9).
     */
    public void setPipeline(int number) {
        getValue("pipeline").setNumber(number);
    }

    /**
     * Helper method to get an entry from the Limelight NetworkTable.
     *
     * @param key - Key for entry.
     * @return NetworkTableEntry of given entry.
     */
    private NetworkTableEntry getValue(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault();
        }

        return table.getTable("limelight").getEntry(key);
    }
}