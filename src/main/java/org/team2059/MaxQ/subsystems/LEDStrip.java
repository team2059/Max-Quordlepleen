package org.team2059.MaxQ.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2059.MaxQ.RobotContainer;

public class LEDStrip extends SubsystemBase {

  private AddressableLED strip;

  private AddressableLEDBuffer buffer;

  private final LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkOrange,
      Color.kBlue);

  public LEDStrip() {
    strip = new AddressableLED(0);

    buffer = new AddressableLEDBuffer(38);

    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();

    gradient.applyTo(buffer);
    strip.setData(buffer);
  }

  @Override
  public void periodic() {
    if (RobotContainer.collector.hasNote()) {
      LEDPattern solidColor = LEDPattern.solid(Color.kRed);
      solidColor.applyTo(buffer);
      strip.setData(buffer);
    } else {
      gradient.applyTo(buffer);
      strip.setData(buffer);
    }
  }
}
