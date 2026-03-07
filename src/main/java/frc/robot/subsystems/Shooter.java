// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax top = new SparkMax(25, MotorType.kBrushless);
  private final SparkMax bottom = new SparkMax(24, MotorType.kBrushless);
  private final SparkMax index = new SparkMax(23, MotorType.kBrushless);
  private final SparkMax hopper = new SparkMax(26, MotorType.kBrushless);
  
  /** Creates a new Shooter. */
  public Shooter() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);
    top.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.smartCurrentLimit(20);
    index.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }
  
  public Command shootTower() {
    return Commands.run(() -> {
      top.set(0.7);
      bottom.set(0.7);
      index.set(1);
      hopper.set(-1); // or -1 depending on how the motor is oriented
    })
    .handleInterrupt(this::stopAll);
  }

  public Command shootHub() {
    return Commands.run(() -> {
      top.set(0.5);
      bottom.set(0.5);
      index.set(1);
      hopper.set(-1);
    })
    .handleInterrupt(this::stopAll);
  }

  public Command shootCorner() {
    return Commands.run(() -> {
      top.set(.85);
      bottom.set(0.85);
      index.set(1);
      hopper.set(-1);
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
