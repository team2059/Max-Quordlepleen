// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

public class TurnToAngleCmd extends Command {
  SwerveBase swerveBase;
  Vision vision;
  final double ANGULAR_P = 2;
  final double ANGULAR_D = 0.0;
  double yaw = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  double rotationSpeed;
  boolean hasTarget;
  double measurement;
  int counter = 0;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleCmd(SwerveBase swerveBase, Vision vision) {
    this.swerveBase = swerveBase;
    this.vision = vision;
    addRequirements(swerveBase, vision);
    // turnController.enableContinuousInput(-180, 180);
    // turnController.setTolerance(0.5, 2.5);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setTolerance(0.04);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = vision.getCamera().getLatestResult();

    hasTarget = result.hasTargets();

    if (hasTarget == false || result.getBestTarget().getPoseAmbiguity() >= 0.2
        || result.getBestTarget().getFiducialId() != 7) {
      this.cancel();
    } else {
      double yaw = result.getBestTarget().getBestCameraToTarget().getY();
      measurement = yaw;
      // swerveBase.getNavX().reset();
    }
    // measurement = swerveBase.getHeading().getDegrees();
    SmartDashboard.putNumber("measurement", measurement);

    rotationSpeed = turnController.calculate(measurement, 0);

    SmartDashboard.putNumber("rotationSpeed", rotationSpeed);

    swerveBase.drive(0, 0, rotationSpeed, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotationSpeed = 0;
    swerveBase.drive(0, 0, 0, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return turnController.atSetpoint() || Math.abs(rotationSpeed) < 0.1;
  }
}