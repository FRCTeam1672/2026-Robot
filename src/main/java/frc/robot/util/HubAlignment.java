package frc.robot.util;

/**
 * Represents a specific alignment configuration for the robot relative to the hub.
 * Uses global pose estimation and trigonometry to calculate the target position.
 * The robot will face the hub from the specified distance with optional lateral offset.
 *
 * @param distanceMeters Distance in meters from the hub center.
 * @param sideOffsetMeters Lateral offset from the direct line to the hub center in meters.
 *                         Positive = left side, Negative = right side (relative to facing the hub).
 */
public record HubAlignment(double distanceMeters, double sideOffsetMeters) {

    /**
     * Create a string representation of this alignment.
     *
     * @return Descriptive string.
     */
    @Override
    public String toString() {
        String sideDescription = sideOffsetMeters > 0 ? "left" : (sideOffsetMeters < 0 ? "right" : "center");
        return String.format("HubAlignment{distance=%.2fm, side=%s, offset=%.2fm}",
                distanceMeters, sideDescription, Math.abs(sideOffsetMeters));
    }

    /**
     * Check if this is a centered approach (no lateral offset).
     *
     * @return True if side offset is approximately zero.
     */
    public boolean isCentered() {
        return Math.abs(sideOffsetMeters) < 0.01;
    }

    /**
     * Get a description of which side this approach is from.
     *
     * @return "left", "right", or "center".
     */
    public String getSideDescription() {
        if (sideOffsetMeters > 0.1) return "left";
        if (sideOffsetMeters < -0.1) return "right";
        return "center";
    }
}
