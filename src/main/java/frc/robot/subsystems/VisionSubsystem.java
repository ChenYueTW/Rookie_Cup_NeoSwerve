package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.PhotonHelper;
import frc.robot.lib.helpers.LimelightHelper;
import frc.robot.lib.math.AprilTagPoseEstimator;
import frc.robot.lib.math.CargoPoseEstimator;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    // TODO
    private static final PhotonHelper cameraHelper = new PhotonHelper();
    // private static final SerialPort serialPort = new SerialPort(115200, Port.kUSB);

    public VisionSubsystem() {
        super("Vision");
    }

    public static Translation2d getAprilTag2d() {
        if (!cameraHelper.hasTarget()) return new Translation2d();
        Translation3d vector = AprilTagPoseEstimator.getAprilTagPose(cameraHelper.getTx(), cameraHelper.getTy(), cameraHelper.getAprilTagId());
        return new Translation2d(vector.getX(), vector.getY());
    }

    public static Translation3d getAprilTag3d() {
        if (!cameraHelper.hasTarget()) return new Translation3d();
        return AprilTagPoseEstimator.getAprilTagPose(cameraHelper.getTx(), cameraHelper.getTy(), cameraHelper.getAprilTagId());
    }

    public static double getAprilTagAngle() {
        return new Translation2d(getAprilTag3d().getX(), getAprilTag3d().getY()).getAngle().getDegrees();
    }

    public static boolean hasTagTarget() {
        return cameraHelper.hasTarget();
    }

    // public static Translation2d getCargo2d() {
    //     if (!hasCargo()) return new Translation2d();
    //     int tx = Integer.valueOf(serialPort.readString().split(",")[0]);
    //     int ty = Integer.valueOf(serialPort.readString().split(",")[1]);
    //     return CargoPoseEstimator.getPositionVector(tx, ty);
    // }

    // public static double getCargoAngle() {
    //     int tx = Integer.valueOf(serialPort.readString().split(",")[0]);
    //     int ty = Integer.valueOf(serialPort.readString().split(",")[1]);
    //     return CargoPoseEstimator.getPositionVector(tx, ty).getAngle().getDegrees();
    // }

    // public static boolean hasCargo() {
    //     return serialPort.readString() != null;
    // }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("AprilTag/Id", cameraHelper.getAprilTagId());
        SmartDashboard.putString("AprilTag/translate", getAprilTag3d().toString());
    }
}
