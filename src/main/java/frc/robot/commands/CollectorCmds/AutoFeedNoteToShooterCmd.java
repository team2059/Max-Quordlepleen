// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCmds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CollectorConstants;
import frc.robot.commands.ShooterCmds.TiltShooterToCollectorCmd;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFeedNoteToShooterCmd extends SequentialCommandGroup {

  Shooter shooter;
  Collector collector;
  double shooterIndexSeped;
  double collectorIndexSpeed;
  BooleanSupplier isNoteInShooter;

  /** Creates a new AlignCollectorToShooterCmd. */
  public AutoFeedNoteToShooterCmd(Collector collector, Shooter shooter, BooleanSupplier isNoteInShooter) {
    this.shooter = shooter;
    this.collector = collector;
    this.isNoteInShooter = isNoteInShooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // addCommands(new MoveCollectorToSetpointCmd(collector,
    // CollectorConstants.collectorTiltAlignToShooterPos)
    // .onlyIf(() -> collector.isNotePresent()));

    // addCommands(new SequentialCommandGroup(new ParallelCommandGroup(new
    // TiltShooterToCollectorCmd(shooter),
    // new MoveCollectorToSetpointCmd(collector,
    // CollectorConstants.collectorTiltAlignToShooterPos))
    // .onlyIf(() -> collector.isNotePresent())),
    // new ConditionalCommand(new InstantCommand(() ->
    // shooter.setIndexMotorSpeed(0)),
    // new InstantCommand(() -> shooter.setIndexMotorSpeed(-0.33))
    // .alongWith(new InstantCommand(() -> collector.setRollerMotor(-0.33)))
    // .onlyIf(() -> !shooter.isNotePresent && collector.isNotePresent()),
    // () -> shooter.isNotePresent));

    // addCommands(new SequentialCommandGroup(new ParallelCommandGroup(new TiltShooterToCollectorCmd(shooter),
    //     ),
    //     new ConditionalCommand(
    //         new InstantCommand(() -> shooter.setIndexMotorSpeed(0))
    //             .alongWith(new InstantCommand(() -> collector.setRollerMotor(0))),
    //         new InstantCommand(() -> shooter.setIndexMotorSpeed(-0.33))
    //             .alongWith(new InstantCommand(() -> collector.setRollerMotor(-0.33)))
    //             .onlyIf(() -> !shooter.isNotePresent && collector.isNotePresent()),
    //         () -> shooter.isNotePresent));

    // addCommands(new ParallelCommandGroup(new TiltShooterToCollectorCmd(shooter),
    // new MoveCollectorToSetpointCmd(collector,
    // CollectorConstants.collectorTiltAlignToShooterPos))
    // .onlyIf(() -> collector.isNotePresent()),
    // new ParallelCommandGroup(new InstantCommand(() ->
    // collector.setRollerMotor(-0.33)),
    // new InstantCommand(() -> shooter.setIndexMotorSpeed(-0.33)))
    // .onlyIf(() -> !shooter.isNotePresent()),
    // new ParallelCommandGroup(new InstantCommand(() ->
    // collector.setRollerMotor(0)),
    // new InstantCommand(() -> shooter.setIndexMotorSpeed(0))));

    addRequirements(collector, shooter);
  }

  public InstantCommand shooterIndex() {

    if (shooter.isNotePresent) {
      shooterIndexSeped = 0;
    } else {
      shooterIndexSeped = -0.33;
    }

    return new InstantCommand(() -> shooter.setIndexMotorSpeed(shooterIndexSeped));

  }

  public InstantCommand collectorIndex() {

    if (shooter.isNotePresent) {
      collectorIndexSpeed = 0;
    } else {
      collectorIndexSpeed = -0.33;
    }

    return new InstantCommand(() -> collector.setRollerMotor(collectorIndexSpeed));

  }

}
