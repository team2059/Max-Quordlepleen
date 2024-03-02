// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmds;

import java.util.Set;

import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterCmds.RunIndexerCmd;
import frc.robot.commands.ShooterCmds.ShootAtRPMsCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoVisionShootCmd extends SequentialCommandGroup {
  Shooter shooter;
  Vision vision;
  double distanceToSpeaker;
  double[] desiredShooterState;
  double desiredShooterVelocity;
  double desiredShooterAngle;

  /** Creates a new VisionShootSequentialCmd. */
  public AutoVisionShootCmd(Shooter shooter, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new ProxyCommand(() -> getCommand()));
    // addCommands(Commands.defer(() -> getCommand(), Set.of(shooter, vision)));
    addCommands(Commands.defer(() -> getCommand(), Set.of(shooter)), new WaitCommand(1),
        new StopAllShooterMotorsCmd(shooter));
  }

  public Command getCommand() {

    distanceToSpeaker = vision.distanceToSpeakerFieldToCamera;

    desiredShooterState = shooter.calculateDesiredShooterState(distanceToSpeaker);
    desiredShooterVelocity = desiredShooterState[0];
    desiredShooterAngle = desiredShooterState[1];

    // return new AutoTiltShooterToSetpointCmd(shooter,
    // desiredShooterState[1])
    // .andThen(
    // new AutoShootAtRPMsCmd(shooter,
    // desiredShooterState[0]));

    return new ParallelDeadlineGroup(new AutoRunIndexerCmd(shooter, desiredShooterVelocity, desiredShooterAngle),
        new AutoTiltShooterToSetpointCmd(shooter, desiredShooterAngle),
        new AutoShootAtRPMsCmd(shooter, desiredShooterVelocity));

  }

}
