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
import frc.robot.Constants;

import frc.robot.Constants.ShootCycle.*;

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

    //lower currentLimit 550
    config.smartCurrentLimit(20);
    index.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }
  
  public Command reverseAgitator() {
    return Commands.run(() -> {
      top.set(0);
      bottom.set(0);
      index.set(0);
      hopper.set(1);
    })
    .handleInterrupt(this::stopAll);
  }

  public Command shootTower() {
    return Commands.sequence(
      reverseAgitator().withTimeout(Constants.outTime),
      Commands.run(() -> {
        top.set(-0.65);
        bottom.set(0.65);
        index.set(-1);
        hopper.set(-1);
      }).withTimeout(Constants.inTime)
    )
    .repeatedly()
    .handleInterrupt(this::stopAll);
  }

  public Command towerAuto() {
    return Commands.sequence(
      reverseAgitator().withTimeout(Constants.outTime),
      Commands.run(() -> {
        top.set(-0.65);
        bottom.set(0.65);
        index.set(-1);
        hopper.set(-1);
      }).withTimeout(Constants.inTime)
    )
    .repeatedly()
    .handleInterrupt(this::stopAll);
  }

  public Command shootHub() {
    return Commands.sequence(
      reverseAgitator().withTimeout(Constants.outTime),
      Commands.run(() -> {
        top.set(-0.5);
        bottom.set(0.5);
        index.set(-1);
        hopper.set(-1);
    }).withTimeout(Constants.inTime)
    )
    .repeatedly()
    .handleInterrupt(this::stopAll);
  }

  public Command shootTrench() {
    return Commands.sequence(
    reverseAgitator().withTimeout(Constants.outTime),
    Commands.run(() -> {
      top.set(-.6);
      bottom.set(0.6);
      index.set(-1);
      hopper.set(-1);
    }).withTimeout(Constants.inTime)
    )
    .repeatedly()
    .handleInterrupt(this::stopAll);
  }

  public Command shootTrenchWall() {
    return Commands.sequence(
    reverseAgitator().withTimeout(Constants.outTime),
    Commands.run(() -> {
      top.set(-.72);
      bottom.set(0.72);
      index.set(-1);
      hopper.set(-1);
    }).withTimeout(Constants.inTime)
    )
    .repeatedly()
    .handleInterrupt(this::stopAll);
  }

  public void stopAll() {
    index.stopMotor();
    top.stopMotor();
    bottom.stopMotor();
    hopper.stopMotor();
  }

  public Command stopCommand() {
    return Commands.run(() -> {
      index.stopMotor();
      top.stopMotor();
      bottom.stopMotor();
      hopper.stopMotor();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
