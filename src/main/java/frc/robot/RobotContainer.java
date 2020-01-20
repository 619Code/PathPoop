package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.drive.ShiftingWCD;
import frc.robot.numbers.Constants;

public class RobotContainer {
    ShiftingWCD sunKist = new ShiftingWCD();
    Joystick driver = new Joystick(0);

    public RobotContainer() {
        System.out.println("Created robot container");
    }

    public ShiftingWCD getDrive() {
        return sunKist;
    }

    public Command getAutonomousCommand() {
        System.out.println("Got auto command");
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts, Units.inchesToMeters(Constants.kvVoltSecondsPerInch),
                        Units.inchesToMeters(Constants.kaVoltSecondsSquaredPerInch)),
                sunKist.kDriveKinematics, 10);
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(Constants.kMaxSpeedFeetPerSecond),
                Units.feetToMeters(Constants.kMaxAccelerationFeetPerSecondSquared))
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(sunKist.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, sunKist::getPose,
                new RamseteController(Constants.kRamseteBMeters, Constants.kRamseteZetaMeters),
                new SimpleMotorFeedforward(Constants.ksVolts, Units.inchesToMeters(Constants.kvVoltSecondsPerInch),
                        Units.inchesToMeters(Constants.kaVoltSecondsSquaredPerInch)),
                sunKist.kDriveKinematics, sunKist::getWheelSpeeds, new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                sunKist::tankDriveVolts, sunKist);

        return ramseteCommand.andThen(() -> sunKist.tankDriveVolts(0, 0));
    }
}
