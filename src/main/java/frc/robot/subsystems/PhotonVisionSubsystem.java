package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
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
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
        if (result.hasTargets()) {
            return true;
        }
    }
    return false;
}

    public Optional<PhotonTrackedTarget> getBestTarget() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
        return Optional.empty();
    }
    PhotonPipelineResult latest = results.get(results.size() - 1);

    return latest.hasTargets()
            ? Optional.of(latest.getBestTarget())
            : Optional.empty();
}
    
    public double getBestTargetYaw() {
        return getBestTarget().map(PhotonTrackedTarget::getYaw).orElse(Double.NaN);
    }
}