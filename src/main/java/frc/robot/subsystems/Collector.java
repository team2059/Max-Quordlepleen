// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {

  // public CANSparkMax collectorRollorMotor;
  // public CANSparkMax elevatorMotor;
  // public DutyCycleEncoder collectorElevatorThruBoreEncoder;

  /** Creates a new Collector. */
  public Collector() {

    // collectorRollorMotor = new CANSparkMax(CollectorConstants.collectorRollerMotorID,
    //     com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    // elevatorMotor = new CANSparkMax(CollectorConstants.elevatorMotorID,
    //     com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    // collectorElevatorThruBoreEncoder = new DutyCycleEncoder(CollectorConstants.collectorElevatorThruBoreEncoderDIO);

  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("collector thrubore pos", collectorElevatorThruBoreEncoder.getAbsolutePosition());

    // This method will be called once per scheduler run
  }
}
