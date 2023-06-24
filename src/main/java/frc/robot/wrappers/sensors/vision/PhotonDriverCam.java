package frc.robot.wrappers.sensors.vision;

import org.photonvision.PhotonCamera;

public class PhotonDriverCam {

    public PhotonDriverCam(final PhotonCamera photonCamera) {
        photonCamera.setDriverMode(true);
    }
}
