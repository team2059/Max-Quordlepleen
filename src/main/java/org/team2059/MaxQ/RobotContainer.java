// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.MaxQ;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team2059.MaxQ.commands.TeleopDriveCmd;
import org.team2059.MaxQ.subsystems.drive.Drivetrain;
import org.team2059.MaxQ.subsystems.drive.GyroIONavX;

import org.team2059.MaxQ.Constants.OperatorConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    public static Drivetrain drivetrain;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static Joystick logitech;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        drivetrain = new Drivetrain(new GyroIONavX());

        logitech = new Joystick(Constants.OperatorConstants.logitechPort);

        drivetrain.setDefaultCommand(
          new TeleopDriveCmd(
            drivetrain,
            () -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
            () -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
            () -> -logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
            () -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis), // slider
            () -> logitech.getRawButton(OperatorConstants.JoystickStrafeOnly), // Strafe Only Button
            () -> logitech.getRawButton(OperatorConstants.JoystickInvertedDrive) // Inverted button
          )
        );

        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        /* RESET NAVX HEADING */
        new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
          .whileTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        /* SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
        new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
          .whileTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return new InstantCommand();
    }
}
