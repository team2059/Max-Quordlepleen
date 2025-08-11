package org.team2059.MaxQ.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.MaxQ.subsystems.shooter.Shooter;


public class SpinupShooterMotorsCmd extends Command {
  private final Shooter shooter;

  public SpinupShooterMotorsCmd(Shooter shooter) {
    this.shooter = shooter;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    shooter.setBothShooterMotorSpeed(0.5);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setBothShooterMotorSpeed(0);
  }
}
