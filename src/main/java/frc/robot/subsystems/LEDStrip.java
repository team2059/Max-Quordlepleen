// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public class RGB {
    public int r, g, b;
    public RGB(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public RGB off = new RGB(0, 0, 0);
  public RGB white = new RGB(255, 255, 255);
  public RGB purple = new RGB(190, 0, 255);
  public RGB yellow = new RGB(255, 255, 0);
  public RGB orange = new RGB(255, 140, 0);

  /** Creates a new LEDStrip. */
  public LEDStrip(int port) {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(10);

    led.setLength(buffer.getLength());
    setOff();
  }

  public void setRGB(RGB target) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, target.r, target.g, target.b);
    }
    led.setData(buffer);
    led.start();
  }

  public void setOff() {
    setRGB(off);
  }
  public void setWhite() {
    setRGB(white);
  }
  public void setPurple() {
    setRGB(purple);
  }
  public void setYellow() {
    setRGB(yellow);
  }
  public void setOrange() {
    setRGB(orange);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
