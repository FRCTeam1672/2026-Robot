// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax top = new SparkMax(25, MotorType.kBrushless);
  private final SparkMax bottom = new SparkMax(24, MotorType.kBrushless);
  private final SparkMax index = new SparkMax(23, MotorType.kBrushless);
  
  /** Creates a new Shooter. */
  public Shooter() {}
  
  public Command shoot() {
    return Commands.run(() -> {
      top.set(0.6);
      bottom.set(0.6);
      index.set(1);
    })
    .handleInterrupt(this::stopAll);
  }

  public void stopAll() {
    index.stopMotor();
    top.stopMotor();
    bottom.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
