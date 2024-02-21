package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final class PowerDistribution {
                public static final int pdpID = 15;
        }

        public static final class ShooterConstants {

                public static final int shooterUpperID = 9; // TODO: CHANGE ALL OF THESE AS NEEDED
                public static final int shooterLowerID = 10;
                public static final int indexerID = 11;
                public static final int shooterTiltID = 12;
                public static final int elevatorID = 13;

                public static final int shooterTiltThruBoreEncoderDIO = 0;

                public static final double alignToCollectorPos = 0; // encoder value for getting note from collector
                public static final double restPos = 0;

                public static final double tiltkP = 0;
                public static final double tiltkD = 0;
                public static final double elevatorkP = 0;
                public static final double elevatorkD = 0;
        }

        public static final class ClimberConstants {
                public static final int climberMotorID = 14; // TODO: change placeholder CAN IDs

                public static final double climbkP = 0;
                public static final double climbkD = 0;
        }

        public static final class CollectorConstants {
                public static final int collectorRollerMotorID = 15;
                public static final int elevatorMotorID = 16;
                public static final int collectorRollerID = 15;
                public static final int collectorTiltID = 16;

                public static final int collectorTiltThruBoreDIO = 9;

                public static final double collectorTiltHomePos = 0; // encoder value for "retracted" collector
                public static final double collectorTiltCollctPos = 0; // encoder value for collecting

                public static final double tiltkD = 0;
                public static final double tiltkP = 0;

        }

        public static final class Presets {

                public static final double PLACE_HOLDER = 0;

        }

        public static final class SwerveModuleConstants {

                // https://yagsl.gitbook.io/yagsl/configuring-yagsl/standard-conversion-factors

                public static final double rotationkP = 0.5;
                public static final double driveEncoderPositionConversionFactor = 0.047286787200699704;
                public static final double rotationEncoderPositionConversionFactor = 16.8 * 0.01745;

                public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.012, 2.7, 0.0);
                public static final double drivekP = 0.1;

        }

        public static final class SwerveBaseConstants {

                public static final double frontLeftAngleOffset = Units.rotationsToRadians(0.29);
                public static final double frontRightAngleOffset = Units.rotationsToRadians(0.117);
                public static final double rearLeftAngleOffset = Units.rotationsToRadians(0.395);
                public static final double rearRightAngleOffset = Units.rotationsToRadians(0);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(18.75);
                public static final double wheelBase = Units.inchesToMeters(24.5);

                // Max module speed, in m/s
                public static final double maxModuleSpeed = 4.5;
                // Drive base radius in meters. Distance from robot center to furthest module.
                public static final double driveBaseRadius = Units.inchesToMeters(11.811);

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

                // used
                public static final double translationkP = 3;
                public static final double rotationkP = 3;

                // Max linear velocity (M/S)
                public static final double maxVelocityMps = 3.0;

                // Max linear acceleration (M/S^2)
                public static final double maxAccelerationMpsSq = 3.0;

                // Max angular velocity (Rad/S)
                public static final double maxAngularVelocityRps = 2 * Math.PI;

                // Max angular acceleration (Rad/S^2)
                public static final double maxAngularAccelerationRpsSq = 4 * Math.PI;

                // max teleop speeds used in TeleopSwerve
                public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;

        }

        public static final class VisionConstants {

                /**
                 * Standard deviations of model states. Increase these numbers to trust your
                 * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
                 * with units in meters and radians, then meters.
                 */

                // values from Team Spectrum 3847’s X-Ray robot from last year
                // https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/5
                public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.1, 0.1, 10);

                /**
                 * Standard deviations of the vision measurements. Increase these numbers to
                 * trust global measurements from vision less. This matrix is in the form
                 * [x, y, theta]ᵀ, with units in meters and radians.
                 */

                // values from Team Spectrum 3847’s X-Ray robot from last year
                public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);

                // Cam mounted facing forward, half a meter forward of center, half a meter up
                // from center.
                public static final Transform3d robotToCam = new Transform3d(
                                new Translation3d(Units.inchesToMeters(14.5), 0.0, Units.inchesToMeters(23)),
                                new Rotation3d(0, 0, 0));

                public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24.5);
                public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(20.5);
                public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
                // x = x offset between robot center (center of the drivetrain) to camera
                // y = y offset between robot center (center of the drivetrain) to camera
                public static final double xCameraOffsetInches = 13.875;
                public static final double yCameraOffsetInches = 0;
                // public static final double originToFront = Units.inchesToMeters(18.75 +
                // 14.625);
                public static final double originToFrontInches = 25; // 27.5

                public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

        }

        public static final class AutoConstants {

        }

}
