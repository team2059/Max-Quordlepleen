// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.MaxQ;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OperatorConstants {

        public static final boolean tuningMode = true;

        /* ===== */
        /* PORTS */
        /* ===== */

        public static final int xboxPort = 0;

        /* ============== */
        /* BUTTONS / AXES */
        /* ============== */

        public static final int XboxResetHeading = 7;
        public static final int XboxFieldRelativeSwitch = 8;
        public static final int XboxSpinupShooterMotors = 6;
        public static final int XboxShooterIndexerAxis = 3; // variable control since this is an axis?
        public static final int XboxIntakeNoteSequence = 1;
        public static final int XboxOuttakeNote = 2;
        public static final int XboxCollectorOutPos = 3;
        public static final int XboxCollectorInPos = 4;
    }

    public static class DrivetrainConstants {

        public static final Distance wheelBase = Inches.of(22.66);
        public static final Distance trackWidth = Inches.of(24.5);

        // Diameter of swerve module wheel
        public static final Distance wheelDiameter = Inches.of(4.0);

        // Kinematics gets each module relative to center. X is forward/backward and Y is left/right.
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front right (+,+)
          new Translation2d(wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0), // back right (+,-)
          new Translation2d(-wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front left (-,+)
          new Translation2d(-wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0) // back left (-,-)
        );

        public static final double driveGearRatio = (6.75);
        public static final double rotationGearRatio = (150.0 / 7.0);

        /* ================== */
        /* CONVERSION FACTORS */
        /* ================== */

        // Given Motor Rotations, convert to Meters traveled
        // (pi * d) / (Gear Ratio)
        // where d is wheel diameter, in meters
        public static final double driveEncoderPositionConversionFactor = 0.0472867872;
        // Given Motor RPM, convert to Meters/second
        public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;
        // Given Motor Rotations, convert to Radians
        // (2 * pi) / (Gear Ratio)
        public static final double rotationEncoderPositionConversionFactor = 0.2932153143;
        // Given Motor RPM, convert to Radians/second
        public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

        /* ============== */
        /* SWERVE MODULES */
        /* ============== */

        /*
         * CAN IDs: found and set via REV hardware client
         * CANcoder Offsets: found in Phoenix Tuner X as "Absolute position"
         *  after manually straightening wheel (converted to radians here by multiplying by 2pi)
         */

        // front left
        public static final int frontLeftDriveMotorId = 1;
        public static final int frontLeftRotationMotorId = 2;
        public static final int frontLeftCanCoderId = 10;
        public static final double frontLeftOffsetRad = 0.915771 * 2 * Math.PI;
        // front right
        public static final int frontRightDriveMotorId = 3;
        public static final int frontRightRotationMotorId = 4;
        public static final int frontRightCanCoderId = 20;
        public static final double frontRightOffsetRad = 0.662109 * 2 * Math.PI;
        // back left
        public static final int backLeftDriveMotorId = 5;
        public static final int backLeftRotationMotorId = 6;
        public static final int backLeftCanCoderId = 30;
        public static final double backLeftOffsetRad = 0.967041 * 2 * Math.PI;
        // back right
        public static final int backRightDriveMotorId = 7;
        public static final int backRightRotationMotorId = 8;
        public static final int backRightCanCoderId = 40;
        public static final double backRightOffsetRad = 0.984131 * 2 * Math.PI;

        /* ======== */
        /* MAXIMUMS */
        /* ======== */

        // Global maximums
        public static final double maxVelocity = 5; // meters/sec
        public static final double maxAcceleration = 10; // meters/sec^2
        public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
        public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
        // Teleop max speeds
        public static final double kTeleDriveMaxSpeed = 4;
        public static final double kTeleDriveMaxAngularSpeed = Math.PI;

        /* =============================== */
        /* SWERVE MODULE CONTROL CONSTANTS */
        /* =============================== */

        public static final double kPRotation = 0.25;
        // kS: voltage needed to overcome static friction
        // kV: voltage needed to run at constant velocity
        // kA: voltage needed to accelerate
        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.17821, 1.9047, 0.14686);
        public static final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0, 0, 0);
    }

    public static class CollectorConstants {
        public static final double collectorOutPos = 0.155;
        public static final double collectorInPos = 0.5;

        public static final double tiltkP = 1.0;
    }
}
