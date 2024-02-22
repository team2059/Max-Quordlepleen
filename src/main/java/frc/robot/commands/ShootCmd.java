// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootCmd extends Command {
  private Shooter shooter;
  private double desiredVelocity;
  double outputVolts;

  /** Creates a new ShootCmd. */
  public ShootCmd(Shooter shooter, double desiredVelocity) {
    this.shooter = shooter;
    this.desiredVelocity = desiredVelocity;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    outputVolts = ShooterConstants.shooterFF.calculate(desiredVelocity);
    outputVolts += shooter.getShooterController()
        .calculate(shooter.shooterUpperMotor.getEncoder().getVelocity(), desiredVelocity);
    shooter.shooterUpperMotor.setVoltage(outputVolts);
    shooter.shooterLowerMotor.setVoltage(outputVolts);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
