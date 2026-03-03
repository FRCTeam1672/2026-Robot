// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final SparkMax intaker = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax driver = new SparkMax(22, MotorType.kBrushless);
  /** Creates a new Indexer. */
  public Intake() {}

  public Command intake() {
    return Commands.run(() -> {
            intaker.set(-1);
        }).handleInterrupt(intaker::stopMotor);
  }

  public Command stop() {
    return Commands.runOnce(() -> {
            intaker.stopMotor();
        });
  }

  public Command up() {
    return Commands.runOnce(() -> driver.set(-0.05))
            .handleInterrupt(driver::stopMotor);
  }

  public Command down() {
    return Commands.runOnce(() -> driver.set(0.05))
            .handleInterrupt(driver::stopMotor);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
