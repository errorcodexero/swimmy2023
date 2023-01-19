package org.xero1425.base.subsystems.tankdrive;

import edu.wpi.first.math.geometry.Pose2d;

/// \file

/// \brief This class represents a look ahead point for the pure pursuit path follower
public class LookAheadPoint {
    // The point on the path a fixed distance ahead of the current robot position
    private Pose2d pose_ ;

    // If true, the point is the end of the path
    private boolean atend_ ;

    /// \brief Create a new look ahead point
    /// \param pose the position on the path ahead of the robot
    /// \param end if true, the point is the end of the path
    public LookAheadPoint(Pose2d pose, boolean end) {
        pose_ = pose ;
        atend_ = end ;
    }

    /// \brief return the pose for the look ahead point
    /// \returns the pose for the look ahead point
    public Pose2d getPose() {
        return pose_ ;
    }

    /// \brief returns true if the point is the end of the path
    /// \returns true if the point is the end of the path
    public boolean atEnd() {
        return atend_ ;
    }
}
