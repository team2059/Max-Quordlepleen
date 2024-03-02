// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoRunIndexerCmd extends Command {
  Shooter shooter;
  double desiredVelocity;
  double desiredAngle;

  /** Creates a new RunIndexerCmd. */
  public AutoRunIndexerCmd(Shooter shooter, double desiredVelocity, double desiredAngle) {
    this.shooter = shooter;
    this.desiredAngle = desiredAngle;
    this.desiredVelocity = desiredVelocity;
    // addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // maybe add some check to see if it's at the desired RPM before firing too

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setIndexMotorSpeed(-0.5);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getVelocity() >= desiredVelocity - 50
        && shooter.getAbsoluteShooterTiltPosDegrees() >= desiredAngle - 1.5;
  }
}
