package org.xero1425.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface IVisionLocalization {
    public class LocationData {
        public Pose3d location ;
        public double when ;
    }

    int getTagCount();
    LocationData getLocation(Pose2d db) ;
    double getDistance() ;
    double getMultiTagDistance() ;
}
