package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
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
import frc.robot.Constants.ShooterRegressionConstants;
import frc.robot.RobotContainer;
import frc.robot.utils.ShooterState;

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
  public PIDController shooterController;

  public DutyCycleEncoder shooterTiltThruBoreEncoder;

  static SplineInterpolator interpolator = new SplineInterpolator();
  static PolynomialSplineFunction velocityFunction = interpolator.interpolate(ShooterRegressionConstants.distances,
      ShooterRegressionConstants.velocities);
  static PolynomialSplineFunction angleFunction = interpolator.interpolate(ShooterRegressionConstants.distances,
      ShooterRegressionConstants.angles);

  public Shooter() {

    // ShooterConstants.velocityMap.put(Units.inchesToMeters(0), 1.0);
    // ShooterConstants.angleMap.put(Units.inchesToMeters(0), 1.0);

    shooterUpperMotor = new CANSparkFlex(Constants.ShooterConstants.shooterUpperID, MotorType.kBrushless);
    shooterLowerMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLowerID, MotorType.kBrushless);

    shooterLowerMotor.follow(shooterUpperMotor);

    indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerID, MotorType.kBrushless);
    shooterTiltMotor = new CANSparkMax(Constants.ShooterConstants.shooterTiltID, MotorType.kBrushless);
    elevatorMotor = new CANSparkMax(Constants.ShooterConstants.elevatorID, MotorType.kBrushless);

    shooterTiltThruBoreEncoder = new DutyCycleEncoder(ShooterConstants.shooterTiltThruBoreEncoderDIO);

    tiltController = new PIDController(ShooterConstants.tiltkP, 0.00,
        ShooterConstants.tiltkD);

    shooterController = new PIDController(ShooterConstants.shooterkP, 0,
        ShooterConstants.shooterkD);

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
    // SmartDashboard.putNumber("shooter thrubore",
    // shooterTiltThruBoreEncoder.getAbsolutePosition());

  }

  public PIDController getShooterController() {
    return shooterController;
  }

  // Method to calculate desired velocity based on distance
  public static double calculateDesiredVelocity(double distance) {

    // Ensure the distance is within the bounds of the original data
    if (distance >= ShooterRegressionConstants.distances[0]
        && distance <= ShooterRegressionConstants.distances[ShooterRegressionConstants.distances.length - 1]) {
      double predictedVelocity = velocityFunction.value(distance);

      // Print the predicted shooter velocity
      System.out.println("Predicted shooter velocity at " + distance + " meters: " + predictedVelocity + " units");
      return predictedVelocity;
    } else {
      // Handle the case where the distance is out of bounds
      System.out.println("Distance is out of the valid range.");
      return 0;
    }

  }

  // Method to calculate desired shooter tilt angle based on distance
  public static double calculateDesiredAngle(double distance) {

    // Ensure the distance is within the bounds of the original data
    if (distance >= ShooterRegressionConstants.distances[0]
        && distance <= ShooterRegressionConstants.distances[ShooterRegressionConstants.distances.length - 1]) {
      double predictedAngle = angleFunction.value(distance);

      // Print the predicted shooter angle
      System.out.println("Predicted shooter angle at " + distance + " meters: " + predictedAngle + " units");
      return predictedAngle;

    } else {
      // Handle the case where the distance is out of bounds
      System.out.println("Distance is out of the valid range.");
      return 0;
    }

  }

  public static ShooterState calculateDesiredShooterState(double distance) {
    double predictedVelocity = 0;
    double predictedAngle = 0;

    // Ensure the distance is within the bounds of the original data
    if (distance >= ShooterRegressionConstants.distances[0]
        && distance <= ShooterRegressionConstants.distances[ShooterRegressionConstants.distances.length - 1]) {
      predictedVelocity = velocityFunction.value(distance);
      predictedAngle = angleFunction.value(distance);

    } else {
      // Handle the case where the distance is out of bounds (will just return a
      // shooter state that has 0 velocity and 0 angle)
      System.out.println("Distance is out of the valid range.");

    }
    // Print the predicted shooter velocity
    System.out
        .println("Predicted shooter velocity at " + distance + " meters: " + predictedVelocity + " units");

    // Print the predicted shooter angle
    System.out.println("Predicted shooter angle at " + distance + " meters: " + predictedAngle + " units");

    return new ShooterState(predictedVelocity, predictedAngle);

  }

}