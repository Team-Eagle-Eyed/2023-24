package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteCamera extends SubsystemBase {

    PhotonCamera camera;

    public NoteCamera(String cameraID) {
        this.camera = new PhotonCamera(cameraID);
    }

    @Override
    public void periodic(){
    
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }
}