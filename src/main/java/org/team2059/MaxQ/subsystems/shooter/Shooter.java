package org.team2059.MaxQ.subsystems.shooter;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  private final SparkFlex upperShooterMotor;
  private final SparkFlexConfig upperShooterMotorConfig = new SparkFlexConfig();

  private final SparkFlex lowerShooterMotor;
  private final SparkFlexConfig lowerShooterMotorConfig = new SparkFlexConfig();

  private final DigitalInput hasNote;

  public Shooter() {

    hasNote = new DigitalInput(2);

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
    return !hasNote.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ShooterHasNote", hasNote());

  }
}
