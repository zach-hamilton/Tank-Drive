// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.SetMotorSpeed;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Drivebase.DriveBaseIOSim;
import frc.robot.subsystems.Drivebase.Drivebase;
import frc.robot.subsystems.Drivebase.DrivebaseIOTalonFX;

import java.util.function.DoubleSupplier;

public class RobotContainer {
  Drivebase drivebase = new Drivebase(new DriveBaseIOSim());
  public RobotContainer() {
    Joystick leftJoystick = new Joystick(1);
    Joystick rightJoystick = new Joystick(0);

    DoubleSupplier leftJoystickSupplier = () -> leftJoystick.getY();
    DoubleSupplier rightJoystickSupplier = () -> rightJoystick.getY();

    SetMotorSpeed motorSpeed = new SetMotorSpeed(drivebase, leftJoystickSupplier, rightJoystickSupplier); 
    
    motorSpeed.addRequirements(drivebase);
    drivebase.setDefaultCommand(motorSpeed);
  }

  public Command getAutonomousCommand() {
    return new TurnToHeading(90, drivebase);
}
}