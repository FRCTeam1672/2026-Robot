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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootCycle;

public class Shooter extends SubsystemBase {
  private final SparkMax top = new SparkMax(24, MotorType.kBrushless);
  private final SparkMax bottom = new SparkMax(25, MotorType.kBrushless);
  private final SparkMax index = new SparkMax(23, MotorType.kBrushless);
  private final SparkMax agitator = new SparkMax(26, MotorType.kBrushless);
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.12, 0.0021); 

  
  /** Creates a new Shooter. */
  @SuppressWarnings("removal")
  public Shooter() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);
    config.inverted(true);
    top.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(false);
    bottom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //lower currentLimit 550
    config.smartCurrentLimit(20);
    index.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    agitator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }
  
  public Command rampUp() {
    return Commands.run(() -> {
      bottom.setVoltage(m_feedforward.calculate(0.25 * 5676));
      top.setVoltage(m_feedforward.calculate(0));
    })
    .handleInterrupt(this::stopAll);
  }

  public Command shootTower() {
    return Commands.sequence(
          Commands.run(() -> {
            top.setVoltage(m_feedforward.calculate(.75*5676));
            bottom.setVoltage(m_feedforward.calculate(0.21 * 5676));
            index.set(-1);
            agitator.set(1);
          }).withTimeout(ShootCycle.inTime)
        ).repeatedly()
      .handleInterrupt(this::stopAll);
  }

  public Command shootTrench() {
    return Commands.sequence(
          Commands.run(() -> {
            top.set(.8);
            bottom.setVoltage(m_feedforward.calculate(0.2 * 5676));
            index.set(-1);
            agitator.set(1);
          }).withTimeout(ShootCycle.inTime)
        ).repeatedly()
      .handleInterrupt(this::stopAll);
  }

  public Command shootPassing() {
   return Commands.sequence(
          Commands.run(() -> {
            top.set(1);
            bottom.setVoltage(m_feedforward.calculate(1 * 5676));
            index.set(-1);
            agitator.set(1);
          }).withTimeout(ShootCycle.inTime)
        ).repeatedly()
      .handleInterrupt(this::stopAll);
  }

  public Command shootCorner() {
   return Commands.sequence(
          Commands.run(() -> {
            top.set(.9);
            bottom.setVoltage(m_feedforward.calculate(0.4 * 5676));
            index.set(-1);
            agitator.set(1);
          }).withTimeout(ShootCycle.inTime)
        ).repeatedly()
      .handleInterrupt(this::stopAll);
  }

  public void stopAll() {
    index.stopMotor();
    top.stopMotor();
    bottom.stopMotor();
    agitator.stopMotor();
  }

  public Command stopCommand() {
    return Commands.run(() -> {
      index.stopMotor();
      top.stopMotor();
      bottom.stopMotor();
      agitator.stopMotor();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Bottom RPM", bottom.getEncoder().getVelocity());
    SmartDashboard.putNumber("Top RPM", top.getEncoder().getVelocity());
  }
}
