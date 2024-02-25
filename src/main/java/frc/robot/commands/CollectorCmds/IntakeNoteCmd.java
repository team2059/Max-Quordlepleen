// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class IntakeNoteCmd extends Command {

  Collector collector;
  double speed;

  /** Creates a new CollectCmd. */
  public IntakeNoteCmd(Collector collector) {
    this.collector = collector;
    this.speed = speed;
    addRequirements(collector);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (collector.isNotePresent()) {
      collector.rollerMotor.set(0);
      CommandScheduler.getInstance()
          .schedule(new FeedNoteToShooterCmd(collector));
    } else {
      collector.setRollerMotor(0.33);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.rollerMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
}
