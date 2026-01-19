package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports necessary to create SwerveDrive object
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// Imports necessary to create telemetry
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;



public class SwerveSubsystem extends SubsystemBase{

    double maximumSpeed = Units.feetToMeters(4.5);

    // Swerve drive object
    private final SwerveDrive swerveDrive; 
    
    
    // Provide swerve configuration file as arguement
    public SwerveSubsystem(File swerveJsonDirectory){
        
        // Set up starting position depending on alliance for odometry
        boolean blueAlliance = false;
        Pose2d startingPose;
        if (blueAlliance){
            // Units are in meters
            startingPose =  new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(0));
        }
        else{
            // Flip for red alliance
            startingPose = new Pose2d(new Translation2d(16, 4), Rotation2d.fromDegrees(180));
        }
        
        // Parse swerve configurations and create swerve drive object
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, startingPose);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        // Configure Swerve Drive
        swerveDrive.setHeadingCorrection(false); // Turn on to correct heading
        swerveDrive.setCosineCompensator(false); // Turn on to automatically slow or speed up swerve modules that should be close to their desired state in theory
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // Tune to compensate for angular skew in movement
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Turn on to periodcally synchronize absolute encoders and motor encoders during periods without movement
    }

    

}
