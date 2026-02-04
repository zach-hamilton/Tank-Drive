package frc.robot.subsystems.Drivebase;

import org.littletonrobotics.junction.AutoLog;

/** Drivebase Interface */
public interface DrivebaseIO {
    @AutoLog
    /** The inputs that we want to track in
    * @param leftVelocity the velocity of the left side of the drivetrain
    * @param leftvoltage the voltage for the motors on the left side of the drivetrain 
    * @param leftCurrent the current of the motors on the left side of the drivetrain
    * @param leftMotorPosition the position of the left side of the drivetrain
    * @param rightVelocity the velocity of the right side of the drivetrain
    * @param rightvoltage the voltage for the motors on the right side of the drivetrain 
    * @param rightCurrent the current of the motors on the right side of the drivetrain
    * @param rightMotorPosition the position of the right side of the drivetrain */
    public static class DrivebaseIOInputs {
        public double leftVelocity = 0.0;
        public double leftVoltage = 0.0;
        public double leftCurrent = 0.0;
        public double leftMotorPosition = 0.0;
        public double rightVelocity = 0.0;
        public double rightVoltage = 0.0;
        public double rightCurrent = 0.0;
        public double rightMotorPosition = 0.0;
        public double heading = 0.0;

    }
    /**
     * Updates the inputs in advantage kit
     * @param inputs
     * the inputs to update
     */
    public default void updateInputs(DrivebaseIOInputs inputs) {}

    /**
     * Set the speed for the motors on the left side of the drivetrain
     * @param speed
     * the speed to set the motor to
     */
    public default void setMotorSpeed(double leftSpeed, double rightSpeed) {}
    
    /**
     * gets the angle of the gyro
     * @return 
     */
    public double getAngle();

    public void resetGyro();
    
    
}
