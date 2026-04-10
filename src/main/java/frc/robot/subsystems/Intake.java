// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HomeConstants.GROUND_INTAKE_HOME_POSITION;
import static frc.robot.Constants.IntakeDownPosition.GROUND_INTAKE_DOWN_POSITION;
import static frc.robot.Constants.GroundPID.GROUND_P;
import static frc.robot.Constants.GroundPID.GROUND_I;
import static frc.robot.Constants.GroundPID.GROUND_D;


public class Intake extends SubsystemBase {

  private final SparkMax intaker = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax driver = new SparkMax(22, MotorType.kBrushless);

  private double groundintakePosition = GROUND_INTAKE_HOME_POSITION;

  /** Creates a new Indexer. */
  @SuppressWarnings("removal")
  public Intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    intaker.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // hopefully this works this time
    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kCoast);
    config.closedLoop.pid(GROUND_P, GROUND_I, GROUND_D);
    config.closedLoop.maxOutput(.4);
    config.closedLoop.minOutput(-.4);
    config.inverted(false);
    driver.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command intake() {
    return Commands.run(() -> {
      intaker.set(-1);
    }).handleInterrupt(intaker::stopMotor);
  }

  public Command reverse() {
    return Commands.run(() -> {
      intaker.set(1);
    }).handleInterrupt(intaker::stopMotor);
  }

  public Command stop() {
    return Commands.runOnce(() -> {
      intaker.stopMotor();
    });
  }

  public boolean isIntakeHomed() {
    return MathUtil.isNear(GROUND_INTAKE_HOME_POSITION, driver.getEncoder().getPosition(), .5);
  }

  public Boolean isIntakeDown() {
    return MathUtil.isNear(GROUND_INTAKE_DOWN_POSITION, driver.getEncoder().getPosition(), .5);
  }

  public Command homeIntake() {
    return Commands.waitUntil(this::isIntakeDown).andThen(
    Commands.runOnce(() -> {
      groundintakePosition = GROUND_INTAKE_HOME_POSITION;
    }));
  }

  public Command intakeDown() {
    return Commands.waitUntil(this::isIntakeHomed).andThen(
    Commands.runOnce(() -> {
      groundintakePosition = GROUND_INTAKE_DOWN_POSITION;
    }));
  }

  // public Command intakeTo(double pos) {
  //   return Commands.runOnce(() -> {
  //     groundintakePosition = pos;
  //   }).andThen(Commands.waitUntil(this::isIntakeAtPosition));
  // }  

  @Override
  public void periodic() {
    driver.getClosedLoopController().setSetpoint(groundintakePosition, ControlType.kPosition);
    SmartDashboard.putNumber("Intake Position", driver.getEncoder().getPosition());
  }
}
