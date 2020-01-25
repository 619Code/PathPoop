package frc.robot.drive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.numbers.Constants;

public class SparkMaxDriveMotors 
{
    public CANSparkMax[] motors;  
    public CANEncoder encoder;
    public CANSparkMax getMasterMotor() {
        return motors[0];
    } 

    public SparkMaxDriveMotors(final int canId1, final int canId2, final int canId3) {
        motors = new CANSparkMax[3];
        motors[0] = CreateNeoSparkMax(canId1);
        motors[1] = CreateNeoSparkMax(canId1);
        motors[2] = CreateNeoSparkMax(canId1);

        motors[1].follow(motors[0]);
        motors[2].follow(motors[0]);

        this.encoder = this.motors[0].getEncoder();        
    }

    private CANSparkMax CreateNeoSparkMax(final int canId) {
        CANSparkMax sparkMax = new CANSparkMax(canId, MotorType.kBrushless);
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setSmartCurrentLimit(Constants.NEO_LIMIT);
        return sparkMax;
    }

    public CANEncoder getEncoder() {
        return this.encoder;
    }

    public double getWheelSpeedInInchesPerSecond() {
        return this.encoder.getVelocity()/Constants.DRIVE_RATIO * Constants.WHEEL_DIAMETER/60;
    }

    public double getEncoderDistanceInInches() {
        return (Constants.WHEEL_DIAMETER * Math.PI * (this.encoder.getPosition() / Constants.ENCODER_TICK_PER_REV));
    }

    public void ResetEncoder() {
        this.encoder.setPosition(-1);
    }
} 