package frc.robot.numbers;

public class Constants {

    // PATHWEAVER
    // Feedforward/Feedback gains (FROM CHARACTERIZATION)
    public static final double ksVolts = 0.216;
    public static final double kvVoltSecondsPerInch = 0.0692;
    public static final double kaVoltSecondsSquaredPerInch = 0.0127;

    public static final double kPDriveVel = 0.601;

    // DifferentialDrive Kinematics
    public static final double kTrackwidthInches = 21;

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedFeetPerSecond = 10;
    public static final double kMaxAccelerationFeetPerSecondSquared = 1.5;

    // Ramsete parameters
    public static final double kRamseteBMeters = 2;
    public static final double kRamseteZetaMeters = 0.7;

    // What even the hell
    public static final boolean kGyroReversed = false;

    // Drive Stuff regular
    public static final int NEO_LIMIT = 60;
    public static final double DRIVE_RATIO = 6.87;

}