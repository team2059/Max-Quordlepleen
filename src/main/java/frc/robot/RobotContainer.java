// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

  /* BUTTON BOX ONE */
  public final ButtonBox buttonBox = new ButtonBox(1);

  /* BUTTON BOX TWO */
  public final ButtonBox buttonBoxTwo = new ButtonBox(2);

  /* LOGITECH */
  public final static Joystick logitech = new Joystick(3);
  private final int kLogitechTranslationAxis = 1;
  private final int kLogitechStrafeAxis = 0;
  private final int kLogitechRotationAxis = 2;
  private final int kLogitechSliderAxis = 3;
  private final int kZeroGyro = 5;
  private final int kFieldOriented = 6;
  private final int kInverted = 1;
  private final int kStrafeOnly = 2;
  private final int kSlowEverything = 3;

  private final JoystickButton zeroGyro = new JoystickButton(logitech, kZeroGyro);

  /* Driver Buttons */

  // private final JoystickButton alignWithTarget = new JoystickButton(driver,
  // XboxController.Button.kRightBumper.value);
  // private final JoystickButton autoBalance = new JoystickButton(driver,
  // XboxController.Button.kX.value);

  /* Subsystems */
  private final SwerveBase swerveBase = new SwerveBase();

  private final ShooterTest shooterTest = new ShooterTest();

  private final Limelight limelight = new Limelight();
  // private final PowerDistributionPanel powerDistributionPanel = new
  // PowerDistributionPanel();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /* Commands */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerveBase.setDefaultCommand(new TeleopSwerve(swerveBase, () -> logitech.getRawAxis(kLogitechTranslationAxis),
        () -> logitech.getRawAxis(kLogitechStrafeAxis), () -> logitech.getRawAxis(kLogitechRotationAxis),
        () -> logitech.getRawAxis(kLogitechSliderAxis),
        () -> !logitech.getRawButton(kFieldOriented),
        () -> logitech.getRawButton(kInverted), () -> logitech.getRawButton(kStrafeOnly),
        () -> logitech.getRawButton(kSlowEverything)));

    // Configure the button bindings
    configureButtonBindings();

    // NamedCommands.registerCommand("arm", new PIDTiltArmCmd(tiltArm, 0.65));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

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

    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().reset()));

    // alignWithTarget.whileTrue(new VisionAlignCmd(limelight, swerveBase));

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

}
