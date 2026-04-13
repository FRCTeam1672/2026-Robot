package frc.robot.util;

/**
 * Represents the hub and its configuration for robot alignment.
 * Uses global pose estimation to calculate alignment angles and positions.
 * AprilTag 7 marks the hub location on the red alliance side, in the left trench, closer to the driver station.
 */
public class Hub {
    /**
     * AprilTag ID for the hub.
     */
    public static final int APRILTAG_ID = 7;

    /**
     * Default distance from the hub center in meters.
     */
    public static final double DEFAULT_DISTANCE_METERS = 1.0;

    /**
     * Create a hub alignment configuration at a specific distance and lateral offset.
     * Uses the robot's global pose to calculate the angle to face the hub.
     *
     * @param distance Distance in meters from the hub center.
     * @param sideOffset Lateral offset from the direct line to the hub center in meters.
     *                   Positive = left side, Negative = right side (relative to facing the hub).
     * @return HubAlignment configuration.
     */
    public static HubAlignment createAlignment(double distance, double sideOffset) {
        return new HubAlignment(distance, sideOffset);
    }

    /**
     * Create a default hub alignment (1 meter away, no lateral offset).
     *
     * @return HubAlignment configuration.
     */
    public static HubAlignment createDefaultAlignment() {
        return createAlignment(DEFAULT_DISTANCE_METERS, 0.0);
    }

    /**
     * Create a left-side hub alignment (approach from the left).
     *
     * @param distance Distance in meters from the hub center.
     * @return HubAlignment configuration.
     */
    public static HubAlignment createLeftAlignment(double distance) {
        return createAlignment(distance, 0.5);  // 0.5m left offset
    }

    /**
     * Create a right-side hub alignment (approach from the right).
     *
     * @param distance Distance in meters from the hub center.
     * @return HubAlignment configuration.
     */
    public static HubAlignment createRightAlignment(double distance) {
        return createAlignment(distance, -0.5);  // 0.5m right offset
    }
}
