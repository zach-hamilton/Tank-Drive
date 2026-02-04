// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivebase;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private final DrivebaseIO io;
  private final DrivebaseIOInputsAutoLogged inputs = new DrivebaseIOInputsAutoLogged();

  /** Creates a new Drivebase. */
  public Drivebase(DrivebaseIO io) {
    this.io = io;
}
  /**
   * Sets the speed for the motors on the left side of the drivetrain
   * @param speed the speed to set the motors to
   */
  public void setMotorSpeed(double leftSpeed, double rightSpeed){
    this.io.setMotorSpeed(leftSpeed, rightSpeed);
    
  }

  public double getLeftMotorPosition() {
    return inputs.leftMotorPosition;
  }

  public double getRightMotorPosition() {
    return inputs.rightMotorPosition;
  }

  public double getYaw(){
    return inputs.heading;
  }

  public double getAngle(){
    return this.io.getAngle();
  }
  public void resetGyro(){
    this.io.resetGyro();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Drivebase", inputs);
  }
}