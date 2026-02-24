// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax top = new SparkMax(21, MotorType.kBrushless); // TODO: fix all IDs
  private final SparkMax bottom = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax index = new SparkMax(21, MotorType.kBrushless);
  
  /** Creates a new Shooter. */
  public Shooter() {}
  
  public void shoot() {
    index.set(1); // TODO: check gregory files
    top.set(1);
    bottom.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
