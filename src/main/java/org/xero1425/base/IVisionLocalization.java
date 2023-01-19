package org.xero1425.base;

import edu.wpi.first.math.geometry.Pose3d;

public interface IVisionLocalization {
    public enum LocationType
    {
        RobotFieldLocation,
        CameraTagLocation,
        CameraFieldLocation
    }

    public class LocationData {
        public int id ;
        public Pose3d location ;
        public LocationType type ;
        public double when ;
    }

    LocationData getLocation() ;
}
