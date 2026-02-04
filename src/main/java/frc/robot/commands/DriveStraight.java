// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase.Drivebase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveStraight extends Command {
  /** Creates a new DriveStraight. */
  double distance;
  double speed;
  Drivebase drivebase;
  final double DIAMETER = 6;
  final double CIRCUMFERENCE = DIAMETER * Math.PI;
  final double GEAR_RATIO = 10.71;
  double motorPositionChangeRadians;
  double startingMotorPositionRadians;
  

  public DriveStraight(double distance, Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
    this.distance = distance;
    this.drivebase = drivebase;
    
    speed = 0.2 * Math.signum(distance);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    System.out.println("Initialized");
    startingMotorPositionRadians = drivebase.getLeftMotorPosition();
    drivebase.setMotorSpeed(speed, -speed);
    double wheelTurns = distance/CIRCUMFERENCE;
    double motorTurns = wheelTurns * GEAR_RATIO;
    motorPositionChangeRadians = motorTurns * (2*Math.PI);
    System.out.println(motorPositionChangeRadians);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setMotorSpeed(0, 0);
    System.out.println("Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distance > 0)  
    {
      return drivebase.getLeftMotorPosition() >= (Math.abs(motorPositionChangeRadians) + startingMotorPositionRadians);
    }
    else if (distance < 0){
      return drivebase.getLeftMotorPosition() <= (motorPositionChangeRadians + startingMotorPositionRadians);
    }
    return false;
  }
}
