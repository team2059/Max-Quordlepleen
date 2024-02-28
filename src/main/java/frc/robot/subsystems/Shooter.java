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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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
    public double currentShooterTiltPosition;

    public RelativeEncoder shooterUpperEncoder;
    public RelativeEncoder shooterLowerEncoder;

    public SparkPIDController shooterUpperController;
    public SparkPIDController shooterLowerController;

    public double currentShooterUpperMotorRPMs;
    public double currentShooterLowerMotorRPMs;

    static SplineInterpolator interpolator = new SplineInterpolator();
    static PolynomialSplineFunction velocityFunction = interpolator.interpolate(ShooterRegressionConstants.distances,
            ShooterRegressionConstants.velocities);
    static PolynomialSplineFunction angleFunction = interpolator.interpolate(ShooterRegressionConstants.distances,
            ShooterRegressionConstants.angles);

    public DigitalInput shooterNoteSensor = new DigitalInput(DIOConstants.shooterOpticalDIO);

    public DutyCycleEncoder shooterTiltThruBoreEncoder = new DutyCycleEncoder(
            DIOConstants.shooterTiltThruBoreEncoderDIO);

    public DigitalInput topElevatorHallEffect = new DigitalInput(ShooterConstants.topElevatorHallEffectDIO);

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
        shooterUpperMotor.setInverted(false);
        shooterUpperEncoder = shooterUpperMotor.getEncoder();

        shooterUpperController = shooterUpperMotor.getPIDController();

        shooterLowerMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLowerID, MotorType.kBrushless);
        shooterLowerEncoder = shooterLowerMotor.getEncoder();
        shooterLowerMotor.setInverted(true);
        shooterLowerController = shooterLowerMotor.getPIDController();

        shooterTiltMotor = new CANSparkMax(Constants.ShooterConstants.shooterTiltID, MotorType.kBrushless);
        shooterTiltMotor.setInverted(true);

        indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerID, MotorType.kBrushless);

        elevatorMotor = new CANSparkMax(Constants.ShooterConstants.elevatorID,
                MotorType.kBrushless);

        elevatorMotor.setInverted(false);

        shooterTiltMotor.setInverted(false);

        shooterUpperController.setP(0.00015);
        shooterUpperController.setI(0);
        shooterUpperController.setD(0);
        shooterUpperController.setFF(0.000155);
        shooterUpperController.setOutputRange(-1, 1);

        // elevatorMotor.enableVoltageCompensation(12);

        shooterLowerController.setP(0.00015);
        shooterLowerController.setI(0);
        shooterLowerController.setD(0);
        shooterLowerController.setFF(0.000155);
        shooterLowerController.setOutputRange(-1, 1);

        // shooterLowerMotor.restoreFactoryDefaults();
        // shooterLowerMotor.setInverted(true);

        // tiltMotor.configFactoryDefault();
        // tiltMotor.setInverted(true);

        // elevatorMotor.configFactoryDefault();
        // elevatorMotor.setInverted(true);

        // shooterTiltThruBoreEncoder.setDistancePerRotation(2 * Math.PI);
        // shooterTiltThruBoreEncoder.setPositionOffset(0);
        shooterTiltThruBoreEncoder.setPositionOffset(0.659);
        shooterTiltThruBoreEncoder.setDistancePerRotation(-360);

    }

    @Override
    public void periodic() {
        currentShooterTiltPosition = getShooterTiltPosDegrees();

        // shooterTiltMotor.set(MathUtil.clamp(MathUtil.applyDeadband(new
        // Joystick(2).getRawAxis(3), 0.05),
        // -0.1, 0.1));

        // if (isNotePresent) {
        // indexerMotor.set(0);
        // }

        SmartDashboard.putBoolean("isTopLimitReached", isTopLimitReached());

        // double tiltSetpoint = ShooterConstants.alignToCollectorPos;

        // double tiltOutput = tiltController.calculate(currentTiltPos, tiltSetpoint);

        // shooterTiltMotor.set(tiltOutput);

        Logger.recordOutput("shooter tilt current absolute pos", getAbsoluteShooterTiltPos());
        Logger.recordOutput("shooter tilt current pos degrees", getShooterTiltPosDegrees());

        // Logger.recordOutput("shooter tilt setpoint", tiltSetpoint);
        // Logger.recordOutput("tiltoutput", tiltOutput);

        currentShooterUpperMotorRPMs = shooterUpperEncoder.getVelocity();
        currentShooterLowerMotorRPMs = shooterLowerEncoder.getVelocity();

        Logger.recordOutput("UpperMotorRPMs", currentShooterUpperMotorRPMs);
        Logger.recordOutput("LowerMotorRPMs ", currentShooterUpperMotorRPMs);

        Logger.recordOutput("shooter optical", isNotePresent());

    }

    public boolean isTopLimitReached() {
        return !topElevatorHallEffect.get();
    }

    public boolean isNotePresent() {
        return !shooterNoteSensor.get();
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

    public void setIndexMotorSpeed(double speed) {
        indexerMotor.set(speed);
    }

    public double getVelocity() {
        return (shooterLowerEncoder.getVelocity() + shooterUpperEncoder.getVelocity()) / 2.0;
    }

    public SparkPIDController getShooterUpperController() {
        return shooterUpperController;
    }

    public double getShooterTiltPosDegrees() {
        return MathUtil.inputModulus(shooterTiltThruBoreEncoder.getDistance(), -90, 90);
    }

    public double getAbsoluteShooterTiltPos() {
        return shooterTiltThruBoreEncoder.getAbsolutePosition();
    }

}