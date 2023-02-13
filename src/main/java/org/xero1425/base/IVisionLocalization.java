package org.xero1425.base;

import edu.wpi.first.math.geometry.Pose3d;

public interface IVisionLocalization {
    public class LocationData {
        public Pose3d location ;
        public double when ;
    }

    int getTagCount();
    LocationData getLocation() ;
    double getDistance() ;
    double getMultiTagDistance() ;
}
