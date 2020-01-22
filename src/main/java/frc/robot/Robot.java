/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.paths.TestPath;
import frc.paths.TurnCalib;
import frc.robot.drive.PathFollower;
import frc.robot.drive.ShiftingWCD;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  static ShiftingWCD drive;
  Joystick test;
  Command m_autonomousCommand;

  @Override
  public void robotInit() {
    drive = new ShiftingWCD();
    test = new Joystick(0);
    m_autonomousCommand = new PathFollower(new TestPath());
  }

  public static ShiftingWCD getDrivetrain() {
    return drive;
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Heading: ", drive.getHeadingDegrees());
    SmartDashboard.putNumber("Left Distance: ", drive.getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Distance: ", drive.getRightEncoderFeet());

    // double P = SmartDashboard.getNumber("P", Constants.DISTANCE_P);
    // double I = SmartDashboard.getNumber("I", Constants.DISTANCE_I);
    // double D = SmartDashboard.getNumber("D", Constants.DISTANCE_D);

    // Constants.DISTANCE_P = P;
    // Constants.DISTANCE_I = I;
    // Constants.DISTANCE_D = D;

  }

  @Override
  public void autonomousInit() {
    drive.resetEncoders();
    drive.resetGyro();
    m_autonomousCommand.start();
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    drive.resetEncoders();
    drive.resetGyro();
  }

  @Override
  public void teleopPeriodic() {
    drive.arcade(test.getRawAxis(1), -test.getRawAxis(4));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
