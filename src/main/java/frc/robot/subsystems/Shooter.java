// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

//import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

  // public CANSparkMax getTiltMotor() {
  // return tiltMotor;
  // }

  // public DutyCycleEncoder getThruBoreEncoder() {
  // return thruBoreEncoder;
  // }

  // public double getThruBorePosition() {
  // return thruBoreEncoder.getAbsolutePosition();
  // }

  // public PIDController getTiltController() {
  // return tiltController;
  // }

  public CANSparkMax shooter1Motor;
  public CANSparkMax shooter2Motor;
  public VictorSPX tiltMotor;
  // public TalonSRX elevatorMotor;

  // public DutyCycleEncoder thruBoreEncoder;
  // public double thruBorePosition;
  // public PIDController tiltController;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {

    shooter1Motor = new CANSparkMax(Constants.ShooterConstants.shooter1ID,
        com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    shooter2Motor = new CANSparkMax(Constants.ShooterConstants.shooter2ID,
        com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    shooter1Motor.setIdleMode(IdleMode.kCoast);
    shooter2Motor.setIdleMode(IdleMode.kCoast);

    tiltMotor = new VictorSPX(Constants.ShooterConstants.tiltID);
    // elevatorMotor = new TalonSRX(Constants.ShooterConstants.elevatorID);

    // thruBoreEncoder = new DutyCycleEncoder(Constants.ArmConstants.thruBoreDIO);

    // tiltController = new PIDController(Constants.ArmConstants.tiltkP, 0.00,
    // Constants.ArmConstants.tiltkD);
    // tiltController.enableContinuousInput(0, 1);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    shooter1Motor.restoreFactoryDefaults();
    shooter1Motor.setInverted(true);

    shooter2Motor.restoreFactoryDefaults();
    shooter2Motor.setInverted(true);

    tiltMotor.configFactoryDefault();
    tiltMotor.setInverted(true);

    // elevatorMotor.configFactoryDefault();
    // elevatorMotor.setInverted(true);

  }

  @Override
  public void periodic() {

    double shooterValue = RobotContainer.logitech.getRawAxis(3); // slider
    // double shooterValue = -0.95;
    // double tiltValue = RobotContainer.controller.getRawAxis(1);
    // value = 0 + ((Math.abs(value - 1)) / 2.0);
    if (Math.abs(shooterValue) <= 0.1)
      shooterValue = 0; // deadband
    // if (Math.abs(tiltValue) <= 0.1)
    // tiltValue = 0; // deadband

    SmartDashboard.putNumber("elevatorValue", shooterValue);

    // elevatorMotor.set(TalonSRXControlMode.PercentOutput, shooterValue);

    // SmartDashboard.putNumber("shooter", shooterValue);
    // SmartDashboard.putNumber("tilt", tiltValue);

    // shooter9.set(shooterValue);
    // shooter11.set(shooterValue);

    // tilt12.set(VictorSPXControlMode.PercentOutput, -tiltValue * 0.4); // super
    // basic manual control
    // SmartDashboard.putNumber("relative tilt pos",
    // tiltMotor.getEncoder().getPosition());

    // SmartDashboard.putNumber("TILTPERCENT", tiltMotor.getAppliedOutput());

    // SmartDashboard.putNumber("TILTVOLTAGE", tiltMotor.getBusVoltage());
    // SmartDashboard.putNumber("thru bore pos",
    // thruBoreEncoder.getAbsolutePosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}