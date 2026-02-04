package frc.robot.subsystems.Drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;



public class DrivebaseIOTalonFX implements DrivebaseIO {
    TalonFX backRightMotor;
    TalonFX frontLeftMotor;
    TalonFX backLeftMotor;
    TalonFX frontRightMotor;

    AHRS gyro;

    private final StatusSignal<AngularVelocity> leftVelocityRotPerSecond; 
    private final StatusSignal<Voltage> leftMotorVoltage;
    private final StatusSignal<Current> leftCurrentAmps;
    private final StatusSignal<Angle> leftMotorAngle;
    private final StatusSignal<AngularVelocity> rightVelocityRotPerSecond;
    private final StatusSignal<Voltage> rightMotorVoltage;
    private final StatusSignal<Current> rightCurrentAmps;
    private final StatusSignal<Angle> rightMotorAngle;
    

    public DrivebaseIOTalonFX() {
        frontRightMotor = new TalonFX(1);
        frontRightMotor.getConfigurator().apply(new TalonFXConfiguration());
        backRightMotor = new TalonFX(2);
        backRightMotor.getConfigurator().apply(new TalonFXConfiguration());
        frontLeftMotor = new TalonFX(3);
        frontLeftMotor.getConfigurator().apply(new TalonFXConfiguration());
        backLeftMotor = new TalonFX(4);
        backLeftMotor.getConfigurator().apply(new TalonFXConfiguration());
        leftVelocityRotPerSecond = frontLeftMotor.getVelocity();
        leftMotorVoltage = frontLeftMotor.getMotorVoltage();
        leftCurrentAmps = frontLeftMotor.getSupplyCurrent();
        leftMotorAngle = frontLeftMotor.getPosition();
        rightVelocityRotPerSecond = frontRightMotor.getVelocity();
        rightMotorVoltage = frontRightMotor.getMotorVoltage();
        rightCurrentAmps = frontRightMotor.getSupplyCurrent();
        rightMotorAngle = frontRightMotor.getPosition();

        gyro = new AHRS(NavXComType.kMXP_SPI);

    }

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        BaseStatusSignal.refreshAll(leftVelocityRotPerSecond, leftMotorVoltage, leftCurrentAmps, leftMotorAngle, rightVelocityRotPerSecond, rightMotorVoltage, rightCurrentAmps, rightMotorAngle);
        inputs.leftVelocity = (leftVelocityRotPerSecond.getValueAsDouble()/3.78) * (2 * Math.PI * Units.inchesToMeters(3));
        inputs.leftVoltage = leftMotorVoltage.getValueAsDouble();
        inputs.leftCurrent = leftCurrentAmps.getValueAsDouble();
        inputs.leftMotorPosition = (rightMotorAngle.getValueAsDouble()/3.78) * (2 * Math.PI * Units.inchesToMeters(3));
        inputs.rightVelocity = (rightVelocityRotPerSecond.getValueAsDouble()/3.78) * (2 * Math.PI * Units.inchesToMeters(3));
        inputs.rightVoltage = rightMotorVoltage.getValueAsDouble();
        inputs.rightCurrent = rightCurrentAmps.getValueAsDouble();
        inputs.rightMotorPosition = (rightMotorAngle.getValueAsDouble()/3.78) * (2 * Math.PI * Units.inchesToMeters(3));
        inputs.heading = gyro.getYaw();
        
    }
    @Override
    public void setMotorSpeed(double leftSpeed, double rightSpeed){
        frontLeftMotor.set(leftSpeed);
        backLeftMotor.set(leftSpeed);
        frontRightMotor.set(rightSpeed);
        backRightMotor.set(rightSpeed);
    }
    @Override
    public double getAngle(){
       return -gyro.getAngle();
    }
    @Override
    public void resetGyro(){
        gyro.reset();
    }

}
