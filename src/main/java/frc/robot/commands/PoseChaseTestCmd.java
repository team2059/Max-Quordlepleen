// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveBaseConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoseChaseTestCmd extends SequentialCommandGroup {
        SwerveBase swerveBase;
        Vision vision;

        public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo
                        .loadAprilTagLayoutField();

        Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d(0, 0, Math.PI));

        private static final int ID_OF_TAG_TO_CHASE = 7;

        Pose2d TAG_FIELD_POSE = new Pose2d(aprilTagFieldLayout.getTagPose(ID_OF_TAG_TO_CHASE).get()
                        .toPose2d().getTranslation(), new Rotation2d(Math.PI));

        // Pose2d endingPose = new Pose2d(aprilTagFieldLayout.getTagPose(7).get()
        // .toPose2d().getTranslation(), new Rotation2d(Math.PI));

        /** Creates a new SequentialChaseTagCmd. */
        public PoseChaseTestCmd(SwerveBase swerveBase,
                        Vision vision) {

                this.vision = vision;
                this.swerveBase = swerveBase;

                addRequirements(vision, swerveBase);
                addCommands(new DeferredCommand(() -> getCommand(), Set.of(swerveBase, vision)),
                                new InstantCommand(() -> swerveBase.stopModules()));
        }

        public Command getCommand() {

                // robot pose
                // var startingPose = new Pose2d(0, 0, new Rotation2d());

                var robotPose2d = swerveBase.getPose();

                var robotPose3d = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0,
                                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

                var result = vision.getCamera().getLatestResult();

                if (result.hasTargets() == false) {
                        return new InstantCommand();
                } else {
                        try {
                                var bestTarget = result.getBestTarget();
                                if (bestTarget.getPoseAmbiguity() >= 0.2
                                                && bestTarget.getFiducialId() != ID_OF_TAG_TO_CHASE) {
                                        return new InstantCommand();
                                } else {

                                        // Get the transformation from the camera to the tag
                                        var camToTarget = bestTarget.getBestCameraToTarget();

                                        // Transform the robot's pose to find the tag's pose
                                        var cameraPose = robotPose3d.transformBy(VisionConstants.robotToCam);
                                        var targetPose = cameraPose.transformBy(camToTarget);

                                        // Transform the tag's pose to set our goal
                                        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                                        Translation2d interiorOne = new Translation2d(robotPose2d.getX()
                                                        + ((goalPose.getX() - robotPose2d.getX()) / 3.0),
                                                        robotPose2d.getY() + ((goalPose.getY() - robotPose2d.getY())
                                                                        / 3.0));
                                        Translation2d interiorTwo = new Translation2d(robotPose2d.getX()
                                                        + (2.0 * (goalPose.getX() - robotPose2d.getX()) / 3.0),
                                                        robotPose2d.getY() + (2.0
                                                                        * (goalPose.getY() - robotPose2d.getY())
                                                                        / 3.0));

                                        var interiorWaypoints = new ArrayList<Translation2d>();
                                        interiorWaypoints.add(interiorOne);
                                        interiorWaypoints.add(interiorTwo);

                                        // Create a list of bezier points from poses. Each pose represents one waypoint.
                                        // The rotation component of the pose should be the direction of travel. Do not
                                        // use holonomic rotation.
                                        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(robotPose2d,
                                                        new Pose2d(interiorOne, interiorOne.getAngle()),
                                                        new Pose2d(interiorTwo, interiorTwo.getAngle()), goalPose);

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
                                                                        goalPose.getRotation().getDegrees())));

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
                                        );
                                }
                        } catch (NullPointerException ex) {
                                ex.printStackTrace();
                                return new InstantCommand();
                        }

                }
        }

}