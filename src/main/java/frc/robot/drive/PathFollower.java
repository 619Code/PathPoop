package frc.robot.drive;

import com.team2363.commands.HelixFollower;
import com.team2363.controller.PIDController;
import com.team319.trajectory.Path;

import frc.robot.numbers.Constants;

public class PathFollower extends HelixFollower {

    private ShiftingWCD sunkist = new ShiftingWCD();

    // These are the 2 PID controllers that will handle error for your total travel
    // distance and heading
    private PIDController headingController = new PIDController(15, 0, 0, 0.001);
    private PIDController distanceController = new PIDController(10, 0, 0, 0.001);

    public PathFollower(Path path) {
        super(path);
        // Make sure to require your subsystem so you don't have conflicting commands
        requires(sunkist);
    }

    @Override
    public void resetDistance() {
        // We need to reset the encoders back to 0 at the start of the path
        sunkist.resetEncoders();
    }

    @Override
    public PIDController getHeadingController() {
        // Here we return the PID controller that we're using to correct the heading
        // error through the path
        return headingController;
    }

    @Override
    public PIDController getDistanceController() {
        // Here we return the PID controller that we're using to correct the distance
        // error through the path
        return distanceController;
    }

    @Override
    public double getCurrentDistance() {
        // Here we need to return the overall robot distance traveled in FEET in this
        // example we are averaging
        // the two sides of the drivetrain to give is the robot's distance travelled
        return (sunkist.getLeftEncoderFeet() + sunkist.getLeftEncoderFeet()) / 2.0;
    }

    @Override
    public double getCurrentHeading() {
        // Here we need to return the current heading of the robot in RADIANS (positive
        // counter-clockwise).
        return Math.toRadians(sunkist.getHeadingRadians());
    }

    @Override
    public void useOutputs(double left, double right) {
        // Here we will use the provided parameters in FPS and send them off to our
        // drivetrain. In this example
        // the max velocity of our drivetrain is 12 FPS. We are dividing the two
        // provided parameters by the max
        // veocity to convert them into a percentage and sending them off to our
        // drivetrain.
        sunkist.setRawPercentOutput(left / Constants.MAX_AUTO_SPEED, right / Constants.MAX_AUTO_SPEED);
    }
}