package frc.robot.lib.helpers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelper {
    DoubleSubscriber aprilTagId;
    DoubleSubscriber ty; 
    DoubleSubscriber tx;

    public LimelightHelper(String tableName) {
        this.aprilTagId = NetworkTableInstance.getDefault().getTable(tableName).getDoubleTopic("tid").subscribe(-1);
        this.tx = NetworkTableInstance.getDefault().getTable(tableName).getDoubleTopic("tx").subscribe(-1);
        this.ty = NetworkTableInstance.getDefault().getTable(tableName).getDoubleTopic("ty").subscribe(-1);
    }

    public int getAprilTagId() {
        return (int) this.aprilTagId.get();
    }

    public double getTx() {
        return this.tx.get();
    }

    public double getTy() {
        return this.ty.get();
    }

    public boolean hasTarget() {
        return this.getAprilTagId() != 0;
    }
}
