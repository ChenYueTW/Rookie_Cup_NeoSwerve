package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.VisionHelper;
import frc.robot.lib.math.AprilTagPoseEstimator;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    // TODO
    private static final VisionHelper visionHelper = new VisionHelper("");

    public VisionSubsystem() {
        super("Vision");
    }

    public static Translation3d getAprilTag3d() {
        if (!visionHelper.hasTarget()) return new Translation3d();
        return AprilTagPoseEstimator.getAprilTagPose(visionHelper.getTx(), visionHelper.getTy(), visionHelper.getAprilTagId());
    }

    public static double getAprilTagAngle() {
        return new Translation2d(getAprilTag3d().getX(), getAprilTag3d().getY()).getAngle().getDegrees();
    }

    public static boolean hasTarget() {
        return visionHelper.hasTarget();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("AprilTag/Id", visionHelper.getAprilTagId());
        SmartDashboard.putString("AprilTag/translate", getAprilTag3d().toString());
    }
}
