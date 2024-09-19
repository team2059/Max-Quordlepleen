// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCmds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDStrip;

public class RotateTeamColorsCmd extends Command {

  private final LEDStrip ledStrip;

  private BooleanSupplier isNotePresent;

  /** Creates a new RotateTeamColorsCmd. */
  public RotateTeamColorsCmd(LEDStrip ledStrip, BooleanSupplier isNotePresent) {

    this.isNotePresent = isNotePresent;
    this.ledStrip = ledStrip;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledStrip.setOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isNotePresent.getAsBoolean()) {
      // NOTE PRESENT - every other LED solid green
      for (int i = 0; i < Constants.LEDConstants.kLEDLength; i++) {
        if (i % 2 == 0) {
          ledStrip.setRGB(i, ledStrip.green.r, ledStrip.green.g, ledStrip.green.b);
        } else {
          ledStrip.setRGB(i, 0, 0, 0);
        }
      }
    } else {
      // TODO: no note: rotate team colors
      // Orange: 36, Blue: 217
      
    }

    ledStrip.setBuffer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledStrip.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
