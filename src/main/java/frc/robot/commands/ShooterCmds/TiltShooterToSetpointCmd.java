// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TiltShooterToSetpointCmd extends Command {
    Shooter shooter;
    double setpoint;
    PIDController tiltController = new PIDController(2, 0, 0.05);

    // ProfiledPIDController profiledPIDController = new ProfiledPIDController(0.1,
    // 0, 0,
    // new TrapezoidProfile.Constraints(0.5, 0.25));

    ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0.1);

    /** Creates a new MoveCollectorCmd. */
    public TiltShooterToSetpointCmd(Shooter shooter, double setpoint) {
        this.shooter = shooter;
        this.setpoint = setpoint;

        addRequirements(shooter);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        tiltController.setTolerance(0.02);

        // profiledPIDController.setGoal(setpoint);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double pidOutout = tiltController.calculate(shooter.getShooterTiltPos(),
                setpoint);

        shooter.shooterTiltMotor.set(pidOutout);

        Logger.recordOutput("voltage output", pidOutout);
        Logger.recordOutput("current tilt pos shooter", shooter.getShooterTiltPos());
        Logger.recordOutput("setpoint", setpoint);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        shooter.shooterTiltMotor.set(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return tiltController.atSetpoint();
    }
}
