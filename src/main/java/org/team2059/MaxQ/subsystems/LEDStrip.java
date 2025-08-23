package org.team2059.MaxQ.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2059.MaxQ.RobotContainer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDStrip extends SubsystemBase {

  private AddressableLED strip;

  private AddressableLEDBuffer buffer;

  private final LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkOrange, Color.kBlue);

  public LEDStrip() {
    strip = new AddressableLED(0);

    buffer = new AddressableLEDBuffer(38);

    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();

    gradient.applyTo(buffer);
//    strip.setData(buffer);
  }

  @Override
  public void periodic() {

  }
}
