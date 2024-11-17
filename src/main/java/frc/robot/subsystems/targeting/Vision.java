package frc.robot.subsystems.targeting;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    PhotonCamera camera;
    
    public Vision(PhotonCamera camera1) {
        camera = camera1;
        PortForwarder.add(5800, "photonvision", 5800);
        camera.setPipelineIndex(0);
    }

    public boolean targetDetected() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            if (result.hasTargets()) {
                return true;
            }
        }
        return false;
    }

    public double getYaw() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        double yaw = 0.0;
        for (PhotonPipelineResult result : results) {
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }
        return yaw;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Target Detected", targetDetected());
    }

}
