package frc.robot.util;

public class Hub {
    public static HubAlignment fromSide(String side) {
        side = side.toLowerCase();
        return switch (side) {
            case "a" -> new HubAlignment(HubOrientation.FRONT, HubSide.LEFT);
            case "b" -> new HubAlignment(HubOrientation.FRONT, HubSide.RIGHT);
            case "c" -> new HubAlignment(HubOrientation.FRONT_RIGHT, HubSide.LEFT);
            case "d" -> new HubAlignment(HubOrientation.FRONT_RIGHT, HubSide.RIGHT);
            case "e" -> new HubAlignment(HubOrientation.BACK_RIGHT, HubSide.LEFT);
            case "f" -> new HubAlignment(HubOrientation.BACK_RIGHT, HubSide.RIGHT);
            case "g" -> new HubAlignment(HubOrientation.BACK, HubSide.LEFT);
            case "h" -> new HubAlignment(HubOrientation.BACK, HubSide.RIGHT);
            case "k" -> new HubAlignment(HubOrientation.BACK_LEFT, HubSide.LEFT);
            case "l" -> new HubAlignment(HubOrientation.BACK_LEFT, HubSide.RIGHT);
            case "i" -> new HubAlignment(HubOrientation.FRONT_LEFT, HubSide.LEFT);
            case "j" -> new HubAlignment(HubOrientation.FRONT_LEFT, HubSide.RIGHT);
            default -> throw new IllegalArgumentException("Illegal side.");
        };

    }
}

enum HubSide {
    LEFT,
    RIGHT
}

enum HubOrientation {

    FRONT(0, 0),
    FRONT_RIGHT(60, 60),
    BACK_RIGHT (120, 120),
    BACK (180, 180),
    FRONT_LEFT (-120, 240),
    BACK_LEFT (-60, 300),
    ;


    private final int robotRotation;
    private final int HubRotation;
    HubOrientation(int robotRotation, int HubRotation) {
        this.robotRotation = robotRotation;
        this.HubRotation = HubRotation;
    }

    public int getRobotRotation() {
        return robotRotation;
    }


    public int getHubRotation() {
        return HubRotation;
    }
}
