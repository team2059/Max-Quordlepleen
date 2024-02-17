package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class Shooter {

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

        tiltController = new PIDController(Constants.ShooterConstants.tiltkP, 0, Constants.ShooterConstants.tiltkD);
        elevatorController = new PIDController(Constants.ShooterConstants.elevatorkP, 0, Constants.ShooterConstants.elevatorkD);

        shooterTiltThruBoreEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterTiltEThruBoreDIO);
    }

    // Getters & setters for all motors, getters for thru bore encoder
    public CANSparkFlex getUpperMotor() {
        return shooterUpperMotor;
    }

    public CANSparkFlex getLowerMotor() {
        return shooterLowerMotor;
    }

    public CANSparkMax getIndexerMotor() {
        return indexerMotor;
    }

    public CANSparkMax getTiltMotor() {
        return shooterTiltMotor;
    }

    public CANSparkMax getElevatorMotor() {
        return elevatorMotor;
    }

    public void setUpperMotor(double output) {
        shooterUpperMotor.set(output);
    }

    public void setLowerMotor(double output) {
        shooterLowerMotor.set(output);
    }

    public void setIndexerMotor(double output) {
        indexerMotor.set(output);
    }

    public void setTiltMotor(double output) {
        shooterTiltMotor.set(output);
    }

    public void setElevatorMotor(double output) {
        elevatorMotor.set(output);
    }

    public DutyCycleEncoder getTiltThruBoreEncoder() {
        return shooterTiltThruBoreEncoder;
    }
    
    public double getTiltThruBoreEncoderPosition() {
        return shooterTiltThruBoreEncoder.getAbsolutePosition();
    }
}
