// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CollectorConstants;
import frc.robot.commands.TeleopSwerveCmd;
import frc.robot.commands.CollectorCmds.IntakeNoteCmd;
import frc.robot.commands.CollectorCmds.TiltCollectorToShooterCmd;
import frc.robot.commands.CollectorCmds.MoveCollectorToSetpointCmd;
import frc.robot.commands.CollectorCmds.PickupNoteCmd;
import frc.robot.commands.ShooterCmds.RunIndexerCmd;
import frc.robot.commands.ShooterCmds.ShootAtRPMsCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToSetpointCmd;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* XBOX CONTROLLER */
  public static final XboxController controller = new XboxController(0);

  /* LOGITECH */
  public final static Joystick logitech = new Joystick(2);

  private final int kLogitechTranslationAxis = 1;
  private final int kLogitechStrafeAxis = 0;
  private final int kLogitechRotationAxis = 2;
  private final int kLogitechSliderAxis = 3;
  private final int kZeroGyro = 5;
  private final int kFieldOriented = 12;
  private final int kInverted = 6; // switch
  private final int kGoToTagButton = 1; // switch
  private final int kStrafeOnly = 2;
  private final int kSlowEverything = 3;

  private final JoystickButton zeroGyro = new JoystickButton(logitech, kZeroGyro);

  private final JoystickButton goToTag = new JoystickButton(logitech, kGoToTagButton);

  private final JoystickButton invertButton = new JoystickButton(logitech, kInverted);

  /* Driver Buttons */

  // private final JoystickButton alignWithTarget = new JoystickButton(driver,
  // XboxController.Button.kRightBumper.value);
  // private final JoystickButton autoBalance = new JoystickButton(driver,
  // XboxController.Button.kX.value);

  /* Subsystems */
  private static final SwerveBase swerveBase = new SwerveBase();

  private static final Collector collector = new Collector();

  private final static Shooter shooter = new Shooter();

  private static final Vision vision = new Vision();
  // private final PowerDistributionPanel powerDistributionPanel = new
  // PowerDistributionPanel();

  SendableChooser<Command> autoChooser;

  boolean isbeinginverted = false;

  TiltShooterToSetpointCmd tiltShooterToSetpointCmd = new TiltShooterToSetpointCmd(shooter, 0.82);

  /* Commands */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    var field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("trajectory").setPoses(poses);
    });

    // NamedCommands.registerCommand("PathfindToTagCmd",
    // new PathfindToTagCmd(swerveBase, vision, 4, 78));

    // Configure the button bindings
    configureButtonBindings();

    // NamedCommands.registerCommand("arm", new PIDTiltArmCmd(tiltArm, 0.65));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // invertButton.toggleOnTrue(new InstantCommand(() -> isbeinginverted =
    // !isbeinginverted)); invert toggle button

    swerveBase.setDefaultCommand(new TeleopSwerveCmd(swerveBase, () -> logitech.getRawAxis(kLogitechTranslationAxis),
        () -> logitech.getRawAxis(kLogitechStrafeAxis), () -> logitech.getRawAxis(kLogitechRotationAxis),
        () -> logitech.getRawAxis(kLogitechSliderAxis),
        () -> !logitech.getRawButton(kFieldOriented),
        () -> logitech.getRawButton(kInverted), () -> logitech.getRawButton(kStrafeOnly),
        () -> logitech.getRawButton(kSlowEverything)));

    // collector.setDefaultCommand(new MoveCollectorToSetpointCmd(collector,
    // CollectorConstants.collectorTiltAlignToShooterPos)
    // .onlyIf(() -> collector.isNotePresent()));

    // shooter.setDefaultCommand(new InstantCommand(() ->
    // shooter.indexerMotor.set(0)));
    // collector.setDefaultCommand(new InstantCommand(() ->
    // collector.rollerMotor.set(0)));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), anxd then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().zeroYaw()));

    // goToTag.whileTrue(new PathfindToTagCmd(swerveBase, vision, 4, 78));

    /* Y - intake up */
    new JoystickButton(controller, 4)
        .onTrue(new TiltCollectorToShooterCmd(collector));

    /* A - intake down */
    new JoystickButton(controller, 1)
        .onTrue(new PickupNoteCmd(collector));

    /* X - intake roller */
    new JoystickButton(controller, 3).whileTrue(new IntakeNoteCmd(collector,
        shooter));

    /* B - outake roller */
    new JoystickButton(controller, 2)
        .whileTrue(new InstantCommand(() -> collector.setRollerMotor(-0.33)))
        .whileFalse(new InstantCommand(() -> collector.setRollerMotor(0)));

    /* left bumper - rev up shooter */
    new JoystickButton(controller, 5)
        .whileTrue(new TiltShooterToSetpointCmd(shooter, 0.83).andThen(new ShootAtRPMsCmd(shooter, 900)));

    /* right bumper - run indexer */
    // new JoystickButton(controller, 6)
    // .whileTrue(
    // new InstantCommand(() -> shooter.setIndexMotorSpeed(-0.33)))
    // .whileFalse(new InstantCommand(() -> shooter.setIndexMotorSpeed(0)));
    new JoystickButton(controller, 6)
        .whileTrue(
            new RunIndexerCmd(shooter));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

  }

  public SwerveBase getSwerveBase() {
    return swerveBase;
  }

  public static Shooter getShooter() {
    return shooter;
  }

  public Vision getVision() {
    return vision;
  }

  public static Collector getCollector() {
    return collector;
  }

}
