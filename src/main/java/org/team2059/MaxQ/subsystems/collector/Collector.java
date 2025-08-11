package org.team2059.MaxQ.subsystems.collector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;
import org.team2059.MaxQ.Constants;
import org.team2059.MaxQ.util.SwerveUtilities;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  private final SparkMax tiltMotor;
  private final SparkMaxConfig tiltMotorConfig = new SparkMaxConfig();

  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  private final DutyCycleEncoder tiltThruBore;

  private final DigitalInput hasNote;

  public Collector() {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.

    tiltMotor = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);
    tiltMotorConfig
      .idleMode(SparkBaseConfig.IdleMode.kBrake);
    tiltMotor.configure(rollerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    rollerMotor = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
    rollerMotorConfig
      .idleMode(SparkBaseConfig.IdleMode.kBrake);
    rollerMotor.configure(rollerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    tiltThruBore = new DutyCycleEncoder(1);

    hasNote = new DigitalInput(0);
  }

  public void setTiltMotorSpeed(double speed) {
    tiltMotor.set(speed);
  }

  public void setRollerMotorSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public boolean hasNote() {
    return hasNote.get();
  }

  public double thruBorePos() {
    return (tiltThruBore.get());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("CollectorNote", hasNote());
    Logger.recordOutput("CollectorThruBore", thruBorePos());
    Logger.recordOutput("CollectorTiltVoltage", tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage());
    Logger.recordOutput("CollectorTiltCurrent", tiltMotor.getOutputCurrent());
    Logger.recordOutput("CollectorTiltPos", tiltMotor.getEncoder().getPosition());
  }
}