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
import frc.robot.numbers.RobotMap;

public class ShiftingWCD extends Subsystem {

  SparkMaxDriveMotors leftMotors;
  SparkMaxDriveMotors rightMotors;

  DifferentialDrive drive;
  DoubleSolenoid shifter;

  AHRS navx;
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public ShiftingWCD() {
    initMotors();
    initDrive();
    initSensors();
    // shifter = new DoubleSolenoid(0, 1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  private void initMotors() {

    leftMotors = new SparkMaxDriveMotors(RobotMap.LEFTMASTER,
                                         RobotMap.LEFTSLAVE0,
                                         RobotMap.LEFTSLAVE1);
    rightMotors = new SparkMaxDriveMotors(RobotMap.RIGHTMASTER,
                                          RobotMap.RIGHTSLAVE0,
                                          RobotMap.RIGHTSLAVE1);
  }

  private void initDrive() {
    drive = new DifferentialDrive(leftMotors.getMasterMotor(), rightMotors.getMasterMotor());
    drive.setSafetyEnabled(false);
  }

  private void initSensors() {
    navx = new AHRS(SPI.Port.kMXP);
    // leftEncoder = leftMotors.getMasterMotor().getEncoder(); leftMaster.getEncoder();
    // rightEncoder = rightMaster.getEncoder();
    resetGyro();
    resetEncoders();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public void resetGyro() {
    navx.reset();
  }

  public void resetEncoders() {
    leftMotors.ResetEncoder();
    rightMotors.ResetEncoder();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  public double getLeftEncoderInches() {
    // return -(Constants.WHEEL_DIAMETER * Math.PI * (leftEncoder.getPosition() /
    // Constants.ENCODER_TICK_PER_REV));
    return -1 * this.leftMotors.getEncoder().getPosition() * 1.7; //Why 1.7?
  }

  public double getRightEncoderInches() {
    // return (Constants.WHEEL_DIAMETER * Math.PI * (rightEncoder.getPosition() /
    // Constants.ENCODER_TICK_PER_REV));
    return this.rightMotors.getEncoder().getPosition() * 1.7; //Why 1.7?
  }

  public double getLeftEncoderFeet() {
    return getLeftEncoderInches() / 12.0;
  }

  public double getRightEncoderFeet() {
    return getRightEncoderInches() / 12.0;
  }

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

}
