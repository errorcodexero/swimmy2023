package frc.robot.subsystems.toplevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldLocationData {

    static final int[] red_grid_tag = new int[] { 3, 2, 1} ;
    static final int[] blue_grid_tag = new int[] { 6, 7, 8 } ;

    public FieldLocationData() {
    }

    public int getLoadingStationTag() {
        int ret = -1 ;

        Alliance a = DriverStation.getAlliance() ;
        if (a == Alliance.Red) {
            ret = 5 ;
        }
        else if (a == Alliance.Blue) {
            ret = 4 ;
        }
        
        return ret ;
    }

    public int getGridTag(int tag) {
        int ret = -1 ;

        Alliance a = DriverStation.getAlliance() ;
        if (a == Alliance.Red) {
            ret = red_grid_tag[tag] ;
        }
        else {
            ret = blue_grid_tag[tag] ;
        }

        return ret;
    }

    public Pose2d getLoadingStationPose(RobotOperation.Slot slot) {
        return new Pose2d() ;
    }

    public Pose2d getGridPose(int tag, RobotOperation.Slot slot) {
        return new Pose2d() ;
    }
}
