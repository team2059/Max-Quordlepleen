package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterRegressionConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    // upper & lower are vortex
    // upper, lower, indexer, shootertilt, elevator
    public CANSparkFlex shooterUpperMotor;
    public CANSparkFlex shooterLowerMotor;
    public CANSparkMax indexerMotor;
    public CANSparkMax shooterTiltMotor;
    public CANSparkMax elevatorMotor;

    public RelativeEncoder shooterUpperEncoder;
    public RelativeEncoder shooterLowerEncoder;

    public PIDController tiltController;
    public PIDController elevatorController;
    // public PIDController shooterController;

    public SparkPIDController shooterController;

    double currentShooterUpperMotorRPMs;
    double desiredShooterUpperMotorRPMs;

    static SplineInterpolator interpolator = new SplineInterpolator();
    static PolynomialSplineFunction velocityFunction = interpolator.interpolate(ShooterRegressionConstants.distances,
            ShooterRegressionConstants.velocities);
    static PolynomialSplineFunction angleFunction = interpolator.interpolate(ShooterRegressionConstants.distances,
            ShooterRegressionConstants.angles);

    public DigitalInput shooterNoteSensor = new DigitalInput(DIOConstants.shooterOpticalDIO);
    public DutyCycleEncoder shooterTiltThruBoreEncoder = new DutyCycleEncoder(
            DIOConstants.shooterTiltThruBoreEncoderDIO);

    public Shooter() {

        shooterUpperMotor = new CANSparkFlex(Constants.ShooterConstants.shooterUpperID, MotorType.kBrushless);
        /**
         * The restoreFactoryDefaults method can be used to reset the configuration
         * parameters
         * in the SPARK MAX to their factory default state. If no argument is passed,
         * these
         * parameters will not persist between power cycles
         */
        // shooterUpperMotor.restoreFactoryDefaults();
        shooterUpperMotor.setInverted(true);
        shooterUpperEncoder = shooterUpperMotor.getEncoder();

        shooterController = shooterUpperMotor.getPIDController();

        shooterLowerMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLowerID, MotorType.kBrushless);
        shooterLowerEncoder = shooterLowerMotor.getEncoder();

        shooterTiltMotor = new CANSparkMax(Constants.ShooterConstants.shooterTiltID, MotorType.kBrushless);

        indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerID, MotorType.kBrushless);

        // elevatorMotor = new CANSparkMax(Constants.ShooterConstants.elevatorID,
        // MotorType.kBrushless);

        tiltController = new PIDController(ShooterConstants.tiltkP, 0.00,
                ShooterConstants.tiltkD);

        shooterController.setP(0);
        shooterController.setI(0);
        shooterController.setD(0);
        shooterController.setOutputRange(-1, 1);

        // tiltController.enableContinuousInput(0, 1);

        // shooterLowerMotor.restoreFactoryDefaults();
        // shooterLowerMotor.setInverted(true);

        // tiltMotor.configFactoryDefault();
        // tiltMotor.setInverted(true);

        // elevatorMotor.configFactoryDefault();
        // elevatorMotor.setInverted(true);

    }

    @Override
    public void periodic() {

        shooterController.setFF(0.00015);

        desiredShooterUpperMotorRPMs = 4500; // max 6784 RPM

        currentShooterUpperMotorRPMs = shooterUpperEncoder.getVelocity();

        Logger.recordOutput("currentUppershooterUpperMotorRPMs ", currentShooterUpperMotorRPMs);
        Logger.recordOutput("desiredshooterUpperMotorRPMs ", desiredShooterUpperMotorRPMs);

        shooterController.setReference(desiredShooterUpperMotorRPMs,
                CANSparkMax.ControlType.kVelocity);

        // Logger.recordOutput("shooter optical", isNotePresent());
        // Logger.recordOutput("shooter tilt", getShooterTiltPos());

        // double sliderValue = RobotContainer.logitech.getRawAxis(3);
        // if (sliderValue > 0) {
        // sliderValue = 0;
        // }

        // sliderValue = MathUtil.clamp(Math.abs(sliderValue), 0, 0.001);
        // Logger.recordOutput("slider value", sliderValue);

        // To measure Ks
        // manually, slowly increase the voltage to the mechanism until it starts to
        // move. The value of
        // is the largest voltage applied before the mechanism begins to move.

        // To tune Kv, increase the velocity feedforward gain
        // until the
        // flywheel approaches
        // the correct
        // setpoint over
        // time. If the
        // flywheel overshoots, reduce Kv.
        // double outputVoltage = new SimpleMotorFeedforward(sliderValue, 0, 0)
        // .calculate(desiredShooterUpperMotorRPMs);

        // shooterUpperMotor.set(outputVoltage);

        // double shooterValue = RobotContainer.logitech.getRawAxis(3); // slider
        // shooterValue = 0 + ((shooterValue - 1) / (2.0) * 0.75);

        // double shooterValue = -0.95;
        // double tiltValue = RobotContainer.controller.getRawAxis(1);
        // value = 0 + ((Math.abs(value - 1)) / 2.0);
        // if (Math.abs(shooterValue) <= 0.1)
        // shooterValue = 0; // deadband
        // if (Math.abs(tiltValue) <= 0.1)
        // tiltValue = 0; // deadband

        // SmartDashboard.putNumber("elevatorValue", shooterValue);

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

    public boolean isNotePresent() {
        return !shooterNoteSensor.get();

    }

    public SparkPIDController getShooterController() {
        return shooterController;
    }

    public double getShooterTiltPos() {
        return shooterTiltThruBoreEncoder.getAbsolutePosition();
    }

    public double[] calculateDesiredShooterState(double distance) {
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

        double[] desiredShooterStateArray = { predictedVelocity, predictedAngle };

        return desiredShooterStateArray;

    }

}