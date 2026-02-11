// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  final SparkMax intake = new SparkMax(21, MotorType.kBrushless);
  public Intake() {
 

    
  }


  
  public void in() 
  {
    intake.set(1);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
