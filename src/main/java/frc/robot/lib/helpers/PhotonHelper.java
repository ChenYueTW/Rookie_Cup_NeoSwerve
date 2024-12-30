package frc.robot.lib.helpers;

import org.photonvision.PhotonCamera;

public class PhotonHelper {
    private final PhotonCamera photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    public PhotonHelper() {
    }

    public double getTx() {
        return this.photonCamera.getLatestResult().getBestTarget().getPitch();
    }

    public double getTy() {
        return this.photonCamera.getLatestResult().getBestTarget().getYaw();
    }

    public int getAprilTagId() {
        if (!this.photonCamera.getLatestResult().hasTargets()) return -1;
        return this.photonCamera.getLatestResult().getBestTarget().getFiducialId();
    }

    public boolean hasTarget() {
        return this.photonCamera.getLatestResult().hasTargets();
    }
}
