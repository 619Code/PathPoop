package frc.robot.numbers;

public class Constants {

    // DRIVE
    public static final double WHEEL_DIAMETER = 4;
    public static final double ENCODER_TICK_PER_REV = 6;
    public static final double MAX_AUTO_SPEED = 6;

    public static final int NEO_LIMIT = 45;
    public static final double DRIVE_RATIO = 6.87;

    // PATHS
    // pid values
    public static final double HEADING_P = 0.64;
    public static final double HEADING_D = 0;
    public static final double HEADING_I = 0;

    public static final double DISTANCE_P = 5;
    public static final double DISTANCE_D = 8;
    public static final double DISTANCE_I = 0;

    // DifferentialDrive Kinematics
    public static final double kTrackwidthInches = 21;

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxAccelerationFeetPerSecondSquared = 1.5;

    // Drive Stuff regular

}