package frc.robot.subsystems.toplevel;

import edu.wpi.first.math.geometry.Pose2d;

public class LocationData {
    public LocationData() {
    }

    public int getLoadingStationTag() {
        return -1 ;
    }

    public int getGridTag(int tag) {
        return -1 ;
    }

    public Pose2d getLoadingStationPose(RobotOperation.Slot slot) {
        return new Pose2d() ;
    }

    public Pose2d getGridPose(int tag, RobotOperation.Slot slot) {
        return new Pose2d() ;
    }
}
