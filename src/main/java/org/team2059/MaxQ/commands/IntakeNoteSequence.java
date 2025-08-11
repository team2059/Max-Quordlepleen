package org.team2059.MaxQ.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team2059.MaxQ.Constants;
import org.team2059.MaxQ.subsystems.collector.Collector;
import org.team2059.MaxQ.subsystems.shooter.Shooter;

public class IntakeNoteSequence extends SequentialCommandGroup {

  public IntakeNoteSequence(Collector collector, Shooter shooter) {

    super(
      new CollectorIntakeNoteCmd(collector),
      new CollectorTiltSetpointCmd(collector, Constants.CollectorConstants.collectorInPos),
      new WaitCommand(0.7),
      new HandoffNoteCmd(collector, shooter)
    );
  }
}