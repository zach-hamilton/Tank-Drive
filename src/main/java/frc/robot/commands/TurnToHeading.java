// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase.Drivebase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToHeading extends Command {
  Drivebase drivebase;
  double targetHeading;
  double speed; 
  double startingAngle;
  double currentangle;
  double error;
  double KP;
  double previousError;
  double KD;
  double I;
  double KI;
  public TurnToHeading(double targetHeading, Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
    this.drivebase = drivebase;
    // targetHeading is in degrees
    this.targetHeading = targetHeading;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingAngle = drivebase.getAngle();
    previousError = targetHeading - drivebase.getAngle();
    I = 0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(drivebase.getAngle());
    double currentangle = drivebase.getAngle();
    double error = targetHeading - (currentangle - startingAngle);
    KP = 0.00271;
    KD = 0.0005;
    KI = 0.000001;
    
    I = I + error;
    double D = previousError - error;
    previousError = error;
    double speed = KP * error + KD * D + KI * I;
    drivebase.setMotorSpeed(-speed, -speed);
    SmartDashboard.putNumber("current_angle", currentangle);
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("target", targetHeading);
    SmartDashboard.putNumber("starting_angle", startingAngle);
    System.out.println("current angle: " + currentangle);
    System.out.println("error: " + error);
    System.out.println("target: " + targetHeading);
    System.out.println("starting angle: " + startingAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setMotorSpeed(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (targetHeading > 0) {
      if (drivebase.getAngle() > startingAngle + targetHeading - 0.5 || drivebase.getAngle() < startingAngle + targetHeading + 0.5);
    }
    else if (targetHeading < 0) {
      if (drivebase.getAngle() > startingAngle + targetHeading - 0.5 || drivebase.getAngle() > startingAngle + targetHeading + 0.5);
    }
    return false;
  }
}
    