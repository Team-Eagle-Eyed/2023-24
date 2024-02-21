package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {

    PhotonCamera camera;

    public Photonvision(String cameraID) {
        this.camera = new PhotonCamera(cameraID);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Has target", camera.getLatestResult().hasTargets());
    }

    public List<PhotonTrackedTarget> getTargets() {
        return camera.getLatestResult().getTargets();
    }
}