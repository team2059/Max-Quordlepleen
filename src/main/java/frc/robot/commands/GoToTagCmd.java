// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveBaseConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToTagCmd extends SequentialCommandGroup {
        SwerveBase swerveBase;
        Limelight limelight;
        double sideOffset;
        double frontOffset;
        int tagId;
        BooleanSupplier whileTrue;

        /** Creates a new SequentialChaseTagCmd. */
        public GoToTagCmd(BooleanSupplier whileTrue, SwerveBase swerveBase,
                        Limelight limelight, double sideOffset, double frontOffset) {
                this.whileTrue = whileTrue;
                this.limelight = limelight;
                this.swerveBase = swerveBase;
                this.sideOffset = sideOffset;
                this.frontOffset = frontOffset;

                addRequirements(limelight, swerveBase);
                addCommands(new ProxyCommand(() -> getCommand()), new InstantCommand(() -> swerveBase.stopModules()));
        }

        public Command getCommand() {

                // robot pose
                var startingPose = new Pose2d(0, 0, new Rotation2d());

                var result = limelight.getCamera().getLatestResult();

                if (result.hasTargets() == false) {
                        return new InstantCommand();
                } else {
                        try {
                                var bestTarget = result.getBestTarget();
                                if (bestTarget.getPoseAmbiguity() >= 0.2) {
                                        return new InstantCommand();
                                } else {
                                        double yawTheta = bestTarget.getBestCameraToTarget().getRotation().getZ();
                                        if (Math.abs(yawTheta) >= 177.5) {
                                                yawTheta = Math.signum(yawTheta) * (180);
                                        }

                                        double xLL = bestTarget.getBestCameraToTarget().getX();
                                        double yLL = bestTarget.getBestCameraToTarget().getY();

                                        // robot final to robot initial rotation (used to rotate vectors in rf frame to
                                        // ri frame)
                                        Rotation2d rf_to_ri = new Rotation2d(yawTheta - Math.PI);

                                        // april tag in robot final coordiante frame
                                        Translation2d A_rf = new Translation2d(
                                                        Units.inchesToMeters(LimelightConstants.originToFrontInches)
                                                                        + Units.inchesToMeters(frontOffset),
                                                        Units.inchesToMeters(sideOffset));
                                        System.out.println("A in robot final" + A_rf.toString());

                                        // april tag in limelight initial coordinate frame
                                        Translation2d A_l0 = new Translation2d(xLL, yLL);
                                        System.out.println("A in limelight initial" + A_l0.toString());

                                        // limelight in robot initial coordinate frame
                                        Translation2d Originl0_rO = new Translation2d(
                                                        Units.inchesToMeters(LimelightConstants.xCameraOffsetInches),
                                                        Units.inchesToMeters(LimelightConstants.yCameraOffsetInches));
                                        System.out.println("Limelight in robot initial" + Originl0_rO.toString());

                                        // april tag in robot initial coordinate frame
                                        Translation2d A_r0 = A_l0.plus(Originl0_rO);
                                        System.out.println("A in robot initial" + A_r0.toString());

                                        Translation2d finalTranslation = A_r0.minus(A_rf.rotateBy(rf_to_ri));
                                        System.out.println(finalTranslation.toString());

                                        Pose2d endingPose = new Pose2d(finalTranslation.getX(),
                                                        finalTranslation.getY(),
                                                        rf_to_ri);

                                        // Pose2d endingPose = new Pose2d(finalTranslation.getX(),
                                        // finalTranslation.getY(),
                                        // new Rotation2d());

                                        Translation2d interiorOne = new Translation2d(endingPose.getX() / 3.0,
                                                        endingPose.getY() / 3.0);
                                        Translation2d interiorTwo = new Translation2d(2.0 * endingPose.getX() / 3.0,
                                                        2.0 * endingPose.getY() / 3.0);

                                        var interiorWaypoints = new ArrayList<Translation2d>();
                                        interiorWaypoints.add(interiorOne);
                                        interiorWaypoints.add(interiorTwo);

                                        System.out.println("START = " + startingPose.toString());
                                        System.out.println("interiorOne" + interiorOne.toString());
                                        System.out.println("interiorTwo" + interiorTwo.toString());
                                        System.out.println("ENDING = " + endingPose.toString());

                                        // Create a list of bezier points from poses. Each pose represents one waypoint.
                                        // The rotation component of the pose should be the direction of travel. Do not
                                        // use holonomic rotation.
                                        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startingPose,
                                                        new Pose2d(interiorOne, interiorOne.getAngle()),
                                                        new Pose2d(interiorTwo, interiorTwo.getAngle()), endingPose);

                                        // Create the path using the bezier points created above

                                        PathPlannerPath path = new PathPlannerPath(
                                                        bezierPoints,
                                                        // The constraints for this path. If using a differential
                                                        // drivetrain, the angular constraints have no effect.
                                                        new PathConstraints(SwerveBaseConstants.maxVelocityMps,
                                                                        SwerveBaseConstants.maxAccelerationMpsSq,
                                                                        SwerveBaseConstants.maxAngularVelocityRps,
                                                                        SwerveBaseConstants.maxAngularAccelerationRpsSq),
                                                        new GoalEndState(0.0, Rotation2d.fromDegrees(
                                                                        endingPose.getRotation().getDegrees())));
                                        // Goal end state. You can set a holonomic rotation here. If using a
                                        // differential drivetrain, the rotation will have no effect.

                                        // not sure if i need to use getPreviewStartingHolonomicPose()

                                        swerveBase.resetOdometry(startingPose);

                                        return new FollowPathHolonomic(
                                                        path,
                                                        swerveBase::getPose, // Robot pose supplier
                                                        swerveBase::getRobotRelativeSpeeds, // ChassisSpeeds supplier.
                                                                                            // MUST BE
                                                                                            // ROBOT RELATIVE
                                                        swerveBase::driveRobotRelative, // Method that will drive the
                                                                                        // robot
                                                        // given ROBOT RELATIVE ChassisSpeeds
                                                        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
                                                                                         // this should likely live in
                                                                                         // your Constants class
                                                                        new PIDConstants(
                                                                                        SwerveBaseConstants.translationkP,
                                                                                        0,
                                                                                        0), // Translation
                                                                        // PID constants
                                                                        new PIDConstants(SwerveBaseConstants.rotationkP,
                                                                                        0,
                                                                                        0), // Rotation
                                                                                            // PID
                                                                        // constants
                                                                        SwerveBaseConstants.maxModuleSpeed, // Max
                                                                                                            // module
                                                                                                            // speed, in
                                                                                                            // m/s
                                                                        SwerveBaseConstants.driveBaseRadius, // Drive
                                                                                                             // base
                                                                        // radius in
                                                                        // meters.
                                                                        // Distance
                                                                        // from robot center to furthest module.
                                                                        new ReplanningConfig() // Default path
                                                                                               // replanning config. See
                                                                                               // the API for the
                                                                                               // options here
                                                        ),
                                                        () -> {
                                                                return false;
                                                        },
                                                        swerveBase // Reference to this subsystem to set requirements
                                        ).onlyWhile(whileTrue);

                                }
                        } catch (NullPointerException ex) {
                                ex.printStackTrace();
                                return new InstantCommand();
                        }

                }

        }
}
