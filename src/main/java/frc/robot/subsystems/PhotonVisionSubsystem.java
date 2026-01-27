package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;

    public PhotonVisionSubsystem(String cameraName) {
        this.camera = new PhotonCamera(cameraName);
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var res = camera.getLatestResult();
        return res.hasTargets() ? Optional.of(res.getBestTarget()) : Optional.empty();
    }

    /** Returns yaw to best target in degrees (positive = target to the right). NaN if no target. */
    public double getBestTargetYaw() {
        return getBestTarget().map(PhotonTrackedTarget::getYaw).orElse(Double.NaN);
    }
}