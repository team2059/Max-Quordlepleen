package org.team2059.MaxQ.subsystems.shooter;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  private final SparkFlex upperShooterMotor;
  private final SparkFlexConfig upperShooterMotorConfig = new SparkFlexConfig();

  private final SparkFlex lowerShooterMotor;
  private final SparkFlexConfig lowerShooterMotorConfig = new SparkFlexConfig();

  // REV color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color noteColor = new Color(153, 76, 19);

  public Shooter() {

    rollerMotor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);
    rollerMotorConfig
      .idleMode(SparkBaseConfig.IdleMode.kBrake);
    rollerMotor.configure(rollerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    upperShooterMotor = new SparkFlex(14, SparkLowLevel.MotorType.kBrushless);
    upperShooterMotorConfig
      .idleMode(SparkBaseConfig.IdleMode.kCoast);
    upperShooterMotor.configure(upperShooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    lowerShooterMotor = new SparkFlex(13, SparkLowLevel.MotorType.kBrushless);
    lowerShooterMotorConfig
      .idleMode(SparkBaseConfig.IdleMode.kCoast)
      .inverted(true);
    lowerShooterMotor.configure(lowerShooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    colorMatcher.addColorMatch(noteColor);
  }

  public void setRollerMotorSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void setUpperShooterMotorSpeed(double speed) {
    upperShooterMotor.set(speed);
  }

  public void setLowerShooterMotorSpeed(double speed) {
    lowerShooterMotor.set(speed);
  }

  public void setBothShooterMotorSpeed(double speed) {
    setUpperShooterMotorSpeed(speed);
    setLowerShooterMotorSpeed(speed);
  }

  public boolean hasNote() {
    Color detectedColor = colorSensor.getColor();

    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    Logger.recordOutput("ColorConfidence", match.confidence);

    return match.confidence >= 0.9;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ShooterNote", hasNote());
    Logger.recordOutput("ColorR", colorSensor.getColor().red);
    Logger.recordOutput("ColorG", colorSensor.getColor().green);
    Logger.recordOutput("ColorB", colorSensor.getColor().blue);
  }
}
