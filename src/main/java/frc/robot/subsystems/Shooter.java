package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

  // upper & lower are vortex
  // upper, lower, indexer, shootertilt, elevator
  public CANSparkFlex shooterUpperMotor;
  public CANSparkFlex shooterLowerMotor;
  public CANSparkMax indexerMotor;
  public CANSparkMax shooterTiltMotor;
  public CANSparkMax elevatorMotor;

  public PIDController tiltController;
  public PIDController elevatorController;

  public DutyCycleEncoder shooterTiltThruBoreEncoder;

  public Shooter() {
    shooterUpperMotor = new CANSparkFlex(Constants.ShooterConstants.shooterUpperID, MotorType.kBrushless);
    shooterLowerMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLowerID, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerID, MotorType.kBrushless);
    shooterTiltMotor = new CANSparkMax(Constants.ShooterConstants.shooterTiltID, MotorType.kBrushless);
    elevatorMotor = new CANSparkMax(Constants.ShooterConstants.elevatorID, MotorType.kBrushless);

    shooterTiltThruBoreEncoder = new DutyCycleEncoder(ShooterConstants.shooterTiltThruBoreEncoderDIO);

    tiltController = new PIDController(ShooterConstants.tiltkP, 0.00,
        ShooterConstants.tiltkD);
    // tiltController.enableContinuousInput(0, 1);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    // shooter1Motor.restoreFactoryDefaults();
    // shooter1Motor.setInverted(false);

    // shooter2Motor.restoreFactoryDefaults();
    // shooter2Motor.setInverted(true);

    // tiltMotor.configFactoryDefault();
    // tiltMotor.setInverted(true);

    // elevatorMotor.configFactoryDefault();
    // elevatorMotor.setInverted(true);

  }

  @Override
  public void periodic() {

    double shooterValue = RobotContainer.logitech.getRawAxis(3); // slider
    shooterValue = 0 + ((shooterValue - 1) / (2.0) * 0.75);

    // double shooterValue = -0.95;
    // double tiltValue = RobotContainer.controller.getRawAxis(1);
    // value = 0 + ((Math.abs(value - 1)) / 2.0);
    if (Math.abs(shooterValue) <= 0.1)
      shooterValue = 0; // deadband
    // if (Math.abs(tiltValue) <= 0.1)
    // tiltValue = 0; // deadband

    SmartDashboard.putNumber("elevatorValue", shooterValue);

    // elevatorMotor.set(TalonSRXControlMode.PercentOutput, shooterValue);

    // SmartDashboard.putNumber("shooter", shooterValue);
    // SmartDashboard.putNumber("tilt", tiltValue);

    // shooter1Motor.set(shooterValue);
    // shooter2Motor.set(shooterValue);
    // intakeMotor.set(shooterValue);

    // tilt12.set(VictorSPXControlMode.PercentOutput, -tiltValue * 0.4); // super
    // basic manual control
    // SmartDashboard.putNumber("relative tilt pos",
    // tiltMotor.getEncoder().getPosition());

    // SmartDashboard.putNumber("TILTPERCENT", tiltMotor.getAppliedOutput());

    // SmartDashboard.putNumber("TILTVOLTAGE", tiltMotor.getBusVoltage());
    // SmartDashboard.putNumber("thru bore pos",
    // thruBoreEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("shooter thrubore", shooterTiltThruBoreEncoder.getAbsolutePosition());

  }

}