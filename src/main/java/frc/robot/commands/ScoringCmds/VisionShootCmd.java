package frc.robot.commands.ScoringCmds;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterCmds.ShootAtRPMsCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToSetpointCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

public class VisionShootCmd extends Command {
    private Shooter shooter;
    private Vision vision;
    private double distanceToSpeaker;
    private double desiredInterpolationVelocity;
    private double desiredInterpolationAngles;

    /** Creates a new ShootCmd. */
    public VisionShootCmd(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter, vision);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        distanceToSpeaker = vision.distanceToSpeakerFieldToCamera;

        double[] desiredShooterState = shooter.calculateDesiredShooterState(distanceToSpeaker);
        double desiredShooterVelocity = desiredShooterState[0];
        double desiredShooterAngle = desiredShooterState[1];

        CommandScheduler.getInstance()
                .schedule(new SequentialCommandGroup(new TiltShooterToSetpointCmd(shooter,
                        desiredShooterAngle)
                        
                        .andThen(
                                new ShootAtRPMsCmd(shooter,
                                        desiredShooterVelocity))));
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
