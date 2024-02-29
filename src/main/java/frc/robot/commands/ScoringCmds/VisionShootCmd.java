package frc.robot.commands.ScoringCmds;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.SwerveBase;
// import frc.robot.subsystems.Vision;

// public class VisionShootCmd extends Command {
//   private Shooter shooter;
//   private Vision vision;
//   private SwerveBase swerveBase;
//   double outputVolts;

//   private double distanceToSpeaker;

//   /** Creates a new ShootCmd. */
//   public VisionShootCmd(Shooter shooter, Vision vision, SwerveBase swerveBase) {
//     this.shooter = shooter;
//     this.vision = vision;
//     this.swerveBase = swerveBase;
//     addRequirements(shooter, vision, swerveBase);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     distanceToSpeaker = vision.getDistancetoSpeaker(swerveBase.getPose());

//     double[] desiredShooterState = shooter.calculateDesiredShooterState(distanceToSpeaker);
//     double desiredShooterVelocity = desiredShooterState[0];
//     double desiredShooterAngle = desiredShooterState[1];

//     outputVolts = ShooterConstants.shooterFF.calculate(desiredShooterVelocity);

//     shooter.shooterUpperMotor.setVoltage(outputVolts);
//     shooter.shooterLowerMotor.setVoltage(outputVolts);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
