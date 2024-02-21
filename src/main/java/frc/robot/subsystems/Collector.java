// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector extends SubsystemBase {

  public CANSparkMax rollerMotor;
  public CANSparkMax tiltMotor;

  public DutyCycleEncoder tiltThruBore;

  public PIDController tiltPidController;

  /** Creates a new Collector. */
  public Collector() {

    rollerMotor = new CANSparkMax(CollectorConstants.collectorRollerID, MotorType.kBrushless);
    tiltMotor = new CANSparkMax(CollectorConstants.collectorTiltID, MotorType.kBrushless);

    tiltThruBore = new DutyCycleEncoder(CollectorConstants.collectorTiltThruBoreDIO);

  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("collector thrubore pos",
    // tiltThruBore.getAbsolutePosition());

    // This method will be called once per scheduler run
  }
}
