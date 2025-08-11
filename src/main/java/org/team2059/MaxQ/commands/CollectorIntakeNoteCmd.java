package org.team2059.MaxQ.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.MaxQ.subsystems.collector.Collector;

public class CollectorIntakeNoteCmd extends Command {
  private final Collector collector;

  private final double intakeSpeed = 0.25;

  public CollectorIntakeNoteCmd(Collector collector) {
    this.collector = collector;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.collector);
  }

  @Override
  public void initialize() {
    collector.setRollerMotorSpeed(intakeSpeed);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return collector.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    collector.setRollerMotorSpeed(0);
  }
}
