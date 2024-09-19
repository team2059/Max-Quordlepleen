// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
  public RGB red = new RGB(255, 0, 0);
  public RGB green = new RGB(0, 255, 0);
  // team colors
  public RGB orange = new RGB(100, 15, 0);
  public RGB blue = new RGB(0, 0, 100);

  /** Creates a new LEDStrip. */
  public LEDStrip(int port, int length) {

    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(buffer.getLength());
    setOff();
  }

  public void setTeamColors(RGB orange, RGB blue) {
    for (int i = 0; i < buffer.getLength(); i++) {
      if ((i % 2) == 0) {
        buffer.setRGB(i, orange.r, orange.g, orange.b);
      } else {
        buffer.setRGB(i, blue.r, blue.g, blue.b);
      }
    }
    led.setData(buffer);
    led.start();
  }

  public void setRGB(int i, int r, int g, int b) {
    buffer.setRGB(i, r, g, b);
  }

  public void setHSV(int i, int h, int s, int v) {
    buffer.setHSV(i, h, s, v);
  }

  public void setBuffer() {
    led.setData(buffer);
  }

  public void setAllRGB(RGB target) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, target.r, target.g, target.b);
    }
    led.setData(buffer);
    led.start();
  }

  public void setEveryOtherRGB(RGB target) {
    for (int i = 0; i < buffer.getLength(); i++) {
      if (i % 2 == 0) buffer.setRGB(i, target.r, target.g, target.b);
    }
    led.setData(buffer);
    led.start();
  }

  public void setOff() {
    for(int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(buffer);
    led.start();
  }

  @Override
  public void periodic() {
  }
}
