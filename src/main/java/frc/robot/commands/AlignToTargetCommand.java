package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class AlignToTargetCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonVisionSubsystem vision;
    private final PIDController pid;

    // TODO: CONFIGURE PID CONTROLS
    private static final double kP = 0.025;
    private static final double kI = 0.0;
    private static final double kD = 0.002;
    private static final double kYawToleranceDeg = 1.5;

    public AlignToTargetCommand(SwerveSubsystem swerve, PhotonVisionSubsystem vision) {
        this.swerve = swerve;
        this.vision = vision;
        this.pid = new PIDController(kP, kI, kD);
        pid.enableContinuousInput(-180.0, 180.0);
        pid.setTolerance(kYawToleranceDeg);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double yawDeg = vision.getBestTargetYaw(); // degrees
            double pidOutput = pid.calculate(yawDeg, 0.0); // radians per second
            pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0); 
            double angularVelocity = pidOutput * swerve.getMaximumChassisAngularVelocity();
            swerve.drive(new Translation2d(0.0, 0.0), angularVelocity, true);
        } else {
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, true);
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && Math.abs(vision.getBestTargetYaw()) <= kYawToleranceDeg;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0.0, 0.0), 0.0, true);
    }
}