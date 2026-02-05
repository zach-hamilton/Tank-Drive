// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivebase;



import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DriveBaseIOSim implements DrivebaseIO {

  private final DCMotor driveGear = new DCMotor(12, 4.69, 257, 1.5, 6380, 2);

  private final DifferentialDrivetrainSim drivetrainSim;
  private double leftSimVoltage;
  private double rightSimVoltage;

  public DriveBaseIOSim() {

    LinearSystem<N2, N2, N2> plant = LinearSystemId.createDrivetrainVelocitySystem(DCMotor.getFalcon500(2), 20, Units.inchesToMeters(3), 
    Units.inchesToMeters(7.5625), 1.72, 3.87);
    Matrix<N7, N1> stdevs = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
    drivetrainSim = new DifferentialDrivetrainSim(plant, driveGear, 3.87, Units.inchesToMeters(3),
      Units.inchesToMeters(15.125), stdevs);
  }

  @Override
  public void setMotorSpeed(double leftSpeed, double rightSpeed) {
    leftSimVoltage = leftSpeed * 12;
    rightSimVoltage = rightSpeed * 12;
    drivetrainSim.setInputs(leftSpeed, rightSpeed);
    System.out.println("speed" + leftSpeed);
  }

  @Override
  public void updateInputs(DrivebaseIOInputs inputs) {
    drivetrainSim.update(0.02);
    inputs.leftVelocity = Units.metersToInches(drivetrainSim.getLeftVelocityMetersPerSecond());
    inputs.leftVoltage = leftSimVoltage;
    inputs.leftCurrent = drivetrainSim.getLeftCurrentDrawAmps();
    inputs.leftMotorPosition = Units.metersToInches(drivetrainSim.getLeftPositionMeters());
    // System.out.println("position " + Units.metersToInches(drivetrainSim.getLeftPositionMeters()));
    inputs.rightVelocity = drivetrainSim.getRightVelocityMetersPerSecond();
    inputs.rightVoltage = rightSimVoltage;
    inputs.rightCurrent = drivetrainSim.getRightCurrentDrawAmps();
    inputs.rightMotorPosition = Units.metersToInches(drivetrainSim.getRightPositionMeters());
  }

  @Override
  public double getAngle() {
    return 0.0;
  }

  @Override
  public void resetGyro() {
  }

}
