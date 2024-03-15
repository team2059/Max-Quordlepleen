// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;

public class LED extends Command {

  public class RGB{
    public int r, g, b;
    public RGB(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public RGB orange = new RGB(255, 165, 0);
  public RGB off = new RGB(0, 0, 0);
  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(3);
    buffer = new AddressableLEDBuffer(60);
    led.setLength(buffer.getLength());
    setOrange();
  }

  public void setRGB(RGB targetRgb) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, targetRgb.r, targetRgb.g, targetRgb.b);
    }

    led.setData(buffer);
  }

  public void reset() {
    setRGB(off);
  }

  public void setOrange() {
    setRGB(orange);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
