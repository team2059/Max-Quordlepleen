// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  public CANSparkMax climberMotor;
  public PIDController climberPIDController;

  /** Creates a new Climber. */
  public Climber() {

    climberMotor = new CANSparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);

    climberPIDController = new PIDController(ClimberConstants.climbkP, 0, ClimberConstants.climbkD);

  }

  public CANSparkMax getClimberMotor() {
    return climberMotor;
  }

  public void setClimberMotor(double output) {
      climberMotor.set(output);
  }

  public PIDController getClimberController() {
      return climberPIDController;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("climber thrubore pos",
    // climberThruBoreEncoder.getAbsolutePosition());

    // This method will be called once per scheduler run
  }
}
