package org.xero1425.base.subsystems.tankdrive;

import edu.wpi.first.math.geometry.Translation2d;

/// \file

/// \brief describes a point along a path that might lie between two segments
public class PathPoint {
    //
    // The first of the two segments that contain the point
    //
    private int which_ ;

    //
    // The percentage the point is between the two segments
    //
    private double pcnt_ ;

    //
    // The location of the point
    //
    private Translation2d loc_ ;

    /// \brief Create a new path point
    public PathPoint(int which, double pcnt, Translation2d loc) {
        which_ = which ;
        pcnt_ = pcnt ;
        loc_ = loc ;
    }

    /// \brief return the first segment this path point derived from
    public int which() {
        return which_ ;
    }

    /// \brief return the percentage the path point is located from the first segment to the second segment
    public double percent() {
        return pcnt_ ;
    }

    /// \brief return the actual location of this path point
    public Translation2d loc() {
        return loc_ ;
    }
}
