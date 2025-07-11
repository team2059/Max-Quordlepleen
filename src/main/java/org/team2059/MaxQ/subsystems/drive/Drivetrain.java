// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.MaxQ.subsystems.drive;

import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import org.team2059.MaxQ.Constants;
import org.team2059.MaxQ.Constants.DrivetrainConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team2059.MaxQ.Constants.DrivetrainConstants.kinematics;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;

  private GyroIO gyro;
  private GyroIOInputsAutoLogged gyroInputs;

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  public Drivetrain(GyroIO gyro) {

    /*
     * Construct four SwerveModules
     * 
     * Arguments: ID, then SwerveModuleIO:
     * - Drive motor can ID
     * - Rotation motor can ID
     * - Cancoder can ID
     * - Cancoder offset in radians
     * - Boolean drive inverter
     * - Boolean rotation inverter
     * - kS, kV, kA constants for drive feedforward (velocity control)
     * - kP constant for drive (velocity control)
     * 
     * 1 frontLeft
     * 2 frontRight
     * 3 backLeft
     * 4 backRight
     */

    frontLeft = new SwerveModule(
        1,
        new SwerveModuleIOReal(
            DrivetrainConstants.frontLeftDriveMotorId,
            DrivetrainConstants.frontLeftRotationMotorId,
            DrivetrainConstants.frontLeftCanCoderId,
            DrivetrainConstants.frontLeftOffsetRad,
            false,
            true,
            0.18707,
            1.972,
            0.2846,
            0.0));
    frontRight = new SwerveModule(
        2,
        new SwerveModuleIOReal(
            DrivetrainConstants.frontRightDriveMotorId,
            DrivetrainConstants.frontRightRotationMotorId,
            DrivetrainConstants.frontRightCanCoderId,
            DrivetrainConstants.frontRightOffsetRad,
            true,
            true,
            0.17367,
            2.0218,
            0.30097,
            0.0));
    backLeft = new SwerveModule(
        3,
        new SwerveModuleIOReal(
            DrivetrainConstants.backLeftDriveMotorId,
            DrivetrainConstants.backLeftRotationMotorId,
            DrivetrainConstants.backLeftCanCoderId,
            DrivetrainConstants.backLeftOffsetRad,
            false,
            true,
            0.1846,
            1.9744,
            0.28488,
            0.0));
    backRight = new SwerveModule(
        4,
        new SwerveModuleIOReal(
            DrivetrainConstants.backRightDriveMotorId,
            DrivetrainConstants.backRightRotationMotorId,
            DrivetrainConstants.backRightCanCoderId,
            DrivetrainConstants.backRightOffsetRad,
            true,
            true,
            0.16226,
            2.0166,
            0.27832,
            0.0));

    // Gyro keeps track of field-relative rotation
    this.gyro = gyro;
    gyroInputs = new GyroIOInputsAutoLogged(); // these inputs allow for us to get values from the gyro
    new Thread(() -> { // gyro may need an extra second to start...
      try {
        Thread.sleep(1000);
        gyro.reset();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }).start();

    // initialize CANcoder offsets
    frontLeft.io.initRotationOffset();
    frontRight.io.initRotationOffset();
    backLeft.io.initRotationOffset();
    backRight.io.initRotationOffset();

    // reset encoders upon each start
    frontLeft.io.resetEncoders();
    frontRight.io.resetEncoders();
    backLeft.io.resetEncoders();
    backRight.io.resetEncoders();

    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getModulePositions());

    SmartDashboard.putData(field);
  }

  /**
   * @return Current robot pose in meters
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Reset odometry to a certain pose,
   * uses current module positions and heading
   * 
   * @param pose specified Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * @return ChassisSpeeds of current robot-relative speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getStates());

    return chassisSpeeds;
  }

  /**
   * Set navX heading to zero
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * @return Rotation2d of current navX heading
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyroInputs.yaw);
  }

  /**
   * @return current swerve module positions in SwerveModulePosition[] array
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeft.inputs.drivePosition,
            new Rotation2d(frontLeft.inputs.rotationAbsolutePositionRadians)),
        new SwerveModulePosition(frontRight.inputs.drivePosition,
            new Rotation2d(frontRight.inputs.rotationAbsolutePositionRadians)),
        new SwerveModulePosition(backLeft.inputs.drivePosition,
            new Rotation2d(backLeft.inputs.rotationAbsolutePositionRadians)),
        new SwerveModulePosition(backRight.inputs.drivePosition,
            new Rotation2d(backRight.inputs.rotationAbsolutePositionRadians))
    };

    return positions;
  }

  /**
   * Method to drive robot-relative
   * 
   * @param chassisSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);
    setModuleStates(newStates);
  }

  /**
   * Method to drive field-relative
   * 
   * @param chassisSpeeds
   */
  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);
    setModuleStates(newStates);
  }

  /**
   * @return current swerve module states of all modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.io.getState();
    states[1] = frontRight.io.getState();
    states[2] = backLeft.io.getState();
    states[3] = backRight.io.getState();

    return states;
  }

  /**
   * Method to set module states
   * 
   * @param desiredStates SwerveModuleState[] desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // makes it never go above specified max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);

    Logger.recordOutput("Desired States", desiredStates);

    // Sets the speed and rotation of each module
    frontLeft.io.setState(desiredStates[0], false);
    frontRight.io.setState(desiredStates[1], false);
    backLeft.io.setState(desiredStates[2], false);
    backRight.io.setState(desiredStates[3], false);
  }

  /**
   * Method to drive the robot either field or robot relative
   * 
   * @param forward
   * @param strafe
   * @param rotation
   * @param isFieldRelative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);

    setModuleStates(states);

  }

  /**
   * Switch fieldRelativeStatus boolean to the opposite value
   */
  public void setFieldRelativity() {
    if (fieldRelativeStatus) {
      fieldRelativeStatus = false;
    } else {
      fieldRelativeStatus = true;
    }
  }

  // Stop all motors in every swerve module
  public void stopAllMotors() {
    frontLeft.io.stop();
    frontRight.io.stop();
    backLeft.io.stop();
    backRight.io.stop();
  }

  // For drivetrain translation routine. Must lock wheels, so instead we use PID
  public void setModulesToZeroRadPID() {
    frontLeft.io.setRotationMotorAnglePID(0);
    frontRight.io.setRotationMotorAnglePID(0);
    backLeft.io.setRotationMotorAnglePID(0);
    backRight.io.setRotationMotorAnglePID(0);
  }

  public void set180GyroRotation(boolean enabled) {
    gyro.set180Rotation(enabled);
  }

  @Override
  public void periodic() {

    // Update gyro inputs & logging
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    // For safety...
    if (DriverStation.isDisabled()) {
      stopAllMotors();
    }

    // Update pose estimator as if it were simply Odometry
    odometry.update(getHeading(), getModulePositions());

    // Logging
    Logger.recordOutput("Pose", getPose());
    field.setRobotPose(getPose());
    Logger.recordOutput("Field-Relative?", fieldRelativeStatus);
    Logger.recordOutput("Real States", getStates());
  }

}
