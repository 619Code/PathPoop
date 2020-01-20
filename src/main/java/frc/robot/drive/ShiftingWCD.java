/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.numbers.Constants;
import frc.robot.numbers.RobotMap;

public class ShiftingWCD extends SubsystemBase {

  CANSparkMax leftMaster, leftSlave0, leftSlave1, rightMaster, rightSlave0, rightSlave1;
  DifferentialDrive drive;
  DoubleSolenoid shifter;

  // PathWeaver
  public final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(Constants.kTrackwidthInches));
  final DifferentialDriveOdometry odometry;
  AHRS navx;
  CANEncoder leftEncoder, rightEncoder;

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public ShiftingWCD() {
    initMotors();
    initDrive();
    initSensors();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    // shifter = new DoubleSolenoid(0, 1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  private void initMotors() {
    leftMaster = new CANSparkMax(10, MotorType.kBrushless);
    leftSlave0 = new CANSparkMax(11, MotorType.kBrushless);
    leftSlave1 = new CANSparkMax(12, MotorType.kBrushless);

    leftSlave0.follow(leftMaster);
    leftSlave1.follow(leftMaster);

    rightMaster = new CANSparkMax(13, MotorType.kBrushless);
    rightSlave0 = new CANSparkMax(14, MotorType.kBrushless);
    rightSlave1 = new CANSparkMax(15, MotorType.kBrushless);

    rightSlave0.follow(rightMaster);
    rightSlave1.follow(rightMaster);

    leftMaster.setSmartCurrentLimit(Constants.NEO_LIMIT);
    leftSlave0.setSmartCurrentLimit(Constants.NEO_LIMIT);
    leftSlave1.setSmartCurrentLimit(Constants.NEO_LIMIT);
    rightMaster.setSmartCurrentLimit(Constants.NEO_LIMIT);
    rightSlave0.setSmartCurrentLimit(Constants.NEO_LIMIT);
    rightSlave1.setSmartCurrentLimit(Constants.NEO_LIMIT);

    brakeMode(leftMaster);
    brakeMode(leftSlave0);
    brakeMode(leftSlave1);
    brakeMode(rightMaster);
    brakeMode(rightSlave0);
    brakeMode(rightSlave1);

    //drive.setMaxOutput(1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  private void initDrive() {
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  private void initSensors() {
    navx = new AHRS(SPI.Port.kMXP);
    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    resetGyro();
    resetEncoders();
  }

  private void resetGyro() {
    navx.reset();
  }

  private void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public double getLeftEncoderInches() {
    return (RobotMap.WHEEL_DIAMETER * Math.PI * (leftEncoder.getPosition() / RobotMap.ENCODER_TICK_PER_REV));
  }

  public double getRightEncoderInches() {
    return (RobotMap.WHEEL_DIAMETER * Math.PI * (rightEncoder.getPosition() / RobotMap.ENCODER_TICK_PER_REV));
  }

  public double getLeftEncoderVelocity() {
    return ((leftEncoder.getVelocity() / Constants.DRIVE_RATIO) * RobotMap.WHEEL_DIAMETER) / 60;
  }

  public double getRightEncoderVelocity() {
    return ((rightEncoder.getVelocity() / Constants.DRIVE_RATIO) * RobotMap.WHEEL_DIAMETER) / 60;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  // Pathweaver
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(getLeftEncoderVelocity()),
        Units.inchesToMeters(getRightEncoderVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderInches() + getRightEncoderInches()) / 2.0;
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public void curve(double speed, double rotation) {
    drive.curvatureDrive(speed, rotation, true);
  }

  public void arcade(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), Units.inchesToMeters(getLeftEncoderInches()),
        Units.inchesToMeters(getRightEncoderInches()));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  private void brakeMode(CANSparkMax mc) {
    mc.setIdleMode(IdleMode.kBrake);
  }
}
