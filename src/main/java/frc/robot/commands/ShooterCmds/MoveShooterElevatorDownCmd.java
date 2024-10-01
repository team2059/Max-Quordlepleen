// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class MoveShooterElevatorDownCmd extends Command {
  Shooter shooter;
  double setPoint;
  double kP, kD, kV, maxVel, maxAcc;
  ProfiledPIDController profiledPIDController;
  // ElevatorFeedforward elevatorFeedforward;
  double currentPosition;
  PIDController pidController;

  /** Creates a new ElevateToAmpCmd. */
  public MoveShooterElevatorDownCmd(Shooter shooter, double setPoint) {
    this.shooter = shooter;
    this.setPoint = setPoint;

    // Constraints Coefficients
    maxVel = 250;
    maxAcc = 125;

    // PID coefficients
    kP = 0.66;
    kD = 0;
    kV = 0.05;

    profiledPIDController = new ProfiledPIDController(kP, 0, kD,
        new TrapezoidProfile.Constraints(maxVel, maxAcc));

    pidController = new PIDController(kP, 0, 0);

    profiledPIDController.setGoal(new State(setPoint, 0));

   // addRequirements(shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    profiledPIDController.setTolerance(1);
    profiledPIDController.reset(shooter.elevatorMotor.getEncoder().getPosition());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      currentPosition = shooter.elevatorMotor.getEncoder().getPosition();

      // goal = final target of mechanism
      double pidOutput = pidController.calculate(currentPosition, setPoint);

      shooter.elevatorMotor.set(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DOWN ENDED!");
    shooter.elevatorMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.elevatorMotor.getEncoder().getPosition() <= 1 || shooter.isBottomLimitReached();
  }
}
