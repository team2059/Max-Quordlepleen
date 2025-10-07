package org.team2059.MaxQ.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.MaxQ.subsystems.collector.Collector;
import org.team2059.MaxQ.subsystems.shooter.Shooter;


public class HandoffNoteCmd extends Command {
  private final Collector collector;
  private final Shooter shooter;

  private final double collectorReleaseSpeed = -0.25;
  private final double shooterIntakeSpeed = 0.25;

  public HandoffNoteCmd(Collector collector, Shooter shooter) {
    this.collector = collector;
    this.shooter = shooter;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.collector, this.shooter);
  }

  @Override
  public void initialize() {
    shooter.setRollerMotorSpeed(shooterIntakeSpeed);
    collector.setRollerMotorSpeed(collectorReleaseSpeed);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return shooter.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRollerMotorSpeed(0);
    collector.setRollerMotorSpeed(0);
  }
}
