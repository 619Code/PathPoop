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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.numbers.Constants;

public class ShiftingWCD extends Subsystem {

  CANSparkMax leftMaster, leftSlave0, leftSlave1, rightMaster, rightSlave0, rightSlave1;
  DifferentialDrive drive;
  DoubleSolenoid shifter;

  AHRS navx;
  CANEncoder leftEncoder, rightEncoder;

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public ShiftingWCD() {
    initMotors();
    initDrive();
    initSensors();
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

  }

  private void initDrive() {
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);
  }

  private void initSensors() {
    navx = new AHRS(SPI.Port.kMXP);
    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();
    resetGyro();
    resetEncoders();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public void resetGyro() {
    navx.reset();
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public double getLeftEncoderInches() {
    // return -(Constants.WHEEL_DIAMETER * Math.PI * (leftEncoder.getPosition() /
    // Constants.ENCODER_TICK_PER_REV));
    return -leftEncoder.getPosition() * 1.7;
  }

  public double getRightEncoderInches() {
    // return (Constants.WHEEL_DIAMETER * Math.PI * (rightEncoder.getPosition() /
    // Constants.ENCODER_TICK_PER_REV));
    return rightEncoder.getPosition() * 1.7;
  }

  public double getLeftEncoderFeet() {
    return getLeftEncoderInches() / 12.0;
  }

  public double getRightEncoderFeet() {
    return getRightEncoderInches() / 12.0;
  }

  // public double getLeftEncoderVelocity() {
  // return ((leftEncoder.getVelocity() / Constants.DRIVE_RATIO) *
  // Constants.WHEEL_DIAMETER) / 60;
  // }

  // public double getRightEncoderVelocity() {
  // return -((rightEncoder.getVelocity() / Constants.DRIVE_RATIO) *
  // Constants.WHEEL_DIAMETER) / 60;
  // }

  public double getHeadingDegrees() {
    return -navx.getAngle();
  }

  public double getHeadingRadians() {
    return Math.toRadians(getHeadingDegrees());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public void curve(double speed, double rotation) {
    drive.curvatureDrive(speed, rotation, true);
  }

  public void arcade(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public void setRawPercentOutput(double left, double right) {
    drive.tankDrive(-left, -right);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
  }

  @Override
  protected void initDefaultCommand() {

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  private void brakeMode(CANSparkMax mc) {
    mc.setIdleMode(IdleMode.kBrake);
  }

}
