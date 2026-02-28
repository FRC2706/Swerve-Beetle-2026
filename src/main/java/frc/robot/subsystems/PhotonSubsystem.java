// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem; // For gyro, gives error because it's on a different branch

// Class
public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1; //declares new camera object, not sure if it should be private or private final
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final double kCameraHeight = 0.44; // assigns camera height in meters
    private static double kTargetHeight = 1.22; // assigns target height in meters // TODO: change this so it changes based on APrilTag ID
    private static final double kCameraPitch = 30*Math.PI/180; // TODO: decide on the final camera pitch, current angle is a guesstimate (30 degrees but in radians)
    private static final double kTargetPitch = Math.PI/2; // assigns target angle in radians
    private final SwerveSubsystem m_SwerveSubsystem;
                            
    public PhotonSubsystem(SwerveSubsystem swerveSubsystem) { //private? or public?
        camera1 = new PhotonCamera("Arducam"); //make sure this name matches the camera name in photonvision interface
        m_SwerveSubsystem = swerveSubsystem;
    }

    @Override
    public void periodic() {
        result = camera1.getLatestResult();

        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.println(target);
        }

        else {
            System.out.println("No target found");
            target = null;

        }
        System.out.println("Pitch:" + getPitch());
        System.out.println("Yaw:" + getYaw());
        System.out.println("Skew" + getSkew());
        System.out.println("Apriltag: " + getTagID());

        Pose2d targetPose = new Pose2d(cameraToTarget().getX(), cameraToTarget().getY(), new Rotation2d(cameraToTarget().getRotation().getZ()));
        Pose2d robotPose = PhotonUtils.estimateFieldToRobot(kCameraHeight, 
                                                            kTargetHeight, 
                                                            kCameraPitch, 
                                                            kTargetPitch, 
                                                            Rotation2d.fromDegrees(-target.getYaw()), 
                                                            m_SwerveSubsystem.getOdometryHeading(), 
                                                            targetPose,
                                                            new Transform2d(cameraToTarget().getTranslation().toTranslation2d(), cameraToTarget().getRotation().toRotation2d())); // SwerveSubsystem.getOdometryHeading() is the gyro angle, gives error because it's on a different branch and should work once merged
            //gyro.getRotation2d()
        System.out.println("Target Pose: " + targetPose);
        System.out.println("Robot Pose: " + robotPose);
        // Calculate robot's field relative pose
        double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose); // Make code publish to networktable so can be put on the thing and driver can see
        System.out.println("Distance to Target: " + distanceToTarget);
    }

    // Returns true if the camera detects an AprilTag
    public boolean hasTarget() {
        return result != null && result.hasTargets();
    }

    // Returns yaw (left/right angle), or 0 if the target is centered
    public double getYaw() {
        if (hasTarget()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    //Returns Skew (angle of the target), or 0 if no target
    public double getSkew() {
        if (hasTarget()) {
            return result.getBestTarget().getSkew();
        }
        return 0.0;
    }

    // Returns pitch (up/down angle), or 0 if no target
    public double getPitch() {
        if (hasTarget()) {
            return result.getBestTarget().getPitch();
        }
        return 0.0;
    }

    // Returns cameraToTarget transform3d things???
    public Transform3d cameraToTarget() {
        if (!hasTarget()) return new Transform3d();
        return new Transform3d(target.getBestCameraToTarget().getTranslation(), target.getBestCameraToTarget().getRotation());
    }

    // Returns AprilTag ID, or -1 if no target
    public int getTagID() {
        if (!hasTarget()) return -1;
        return target.getFiducialId();
    }

}