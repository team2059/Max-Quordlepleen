package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final class PowerDistribution {
                public static final int pdpID = 15;
        }

        public static final class ShooterConstants {

                public static final int shooter1ID = 9;
                public static final int shooter2ID = 10;
                public static final int intakeID = 11;
                public static final int tiltID = 12;
                public static final int elevatorID = 13;

        }

        public static final class Presets {

                public static final double PLACE_HOLDER = 0;

        }

        public static final class Swerve {

                public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);
                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(18.75);
                public static final double wheelBase = Units.inchesToMeters(24.5);

                // nominal (real) divided by fudge factor
                public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.01085);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
                public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
                                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
                );

                /* Swerve Profiling Values */
                public static final double maxSpeed = 4.5; // meters per second
                public static final double maxAngularVelocity = 11.5;

                public static final int frontLeftRotationMotorId = 7;
                public static final int frontLeftDriveMotorId = 8;

                public static final int frontRightRotationMotorId = 5;
                public static final int frontRightDriveMotorId = 6;

                public static final int rearLeftRotationMotorId = 3;
                public static final int rearLeftDriveMotorId = 4;

                public static final int rearRightRotationMotorId = 1;
                public static final int rearRightDriveMotorId = 2;

                public static final int frontLeftRotationEncoderId = 12;
                public static final int frontRightRotationEncoderId = 13;
                public static final int rearLeftRotationEncoderId = 14;
                public static final int rearRightRotationEncoderId = 11;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

                public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

        }

        public static final class LimelightConstants {
                // x = x offset between robot center (center of the drivetrain) to camera
                // y = y offset between robot center (center of the drivetrain) to camera
                public static final double xCameraOffsetInches = 13.875;
                public static final double yCameraOffsetInches = 0;
                // public static final double originToFront = Units.inchesToMeters(18.75 +
                // 14.625);
                public static final double originToFrontInches = 25; // 27.5
        }

        public static final class GoToTagConstants {

                // Max linear velocity (M/S)
                public static final double maxVelocityMps = 3.0;

                // Max linear acceleration (M/S^2)
                public static final double maxAccelerationMpsSq = 3.0;

                // Max angular velocity (Rad/S)
                public static final double maxAngularVelocityRps = 2 * Math.PI;

                // Max angular acceleration (Rad/S^2)
                public static final double maxAngularAccelerationRpsSq = 4 * Math.PI;

                public static final double translationkP = 3;
                public static final double translationkI = 0;
                public static final double translationkD = 0;

                public static final double rotationkP = 3;
                public static final double rotationkI = 0;
                public static final double rotationkD = 0;

                // Max module speed, in m/s
                public static final double maxModuleSpeed = 3.5;
                // Drive base radius in meters. Distance from robot center to furthest module.
                public static final double driveBaseRadius = Units.inchesToMeters(11.811);
        }

        public static final class AutoConstants {

        }

}
