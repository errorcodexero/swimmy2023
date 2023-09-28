package org.xero1425.misc ;

/// \file

/// \brief This class represents a single segment of a path.
/// A segment consits of the x and y position, the distance along the path, the velocity,
/// acceleration, and jerk for the path, and the heading.
public class XeroPathSegment
{
    private double time_ ;
    private double x_ ;
    private double y_ ;
    private double pos_ ;
    private double vel_ ;
    private double accel_ ;
    private double jerk_ ;
    private double heading_ ;
    private double curvature_ ;
    private double rotation_ ;

    /// \brief create a new path segment
    /// \param time the time since the start of this path for this segment
    /// \param x the x coordinate relative to the start point of the path
    /// \param y the y coordinate relative to the start point of the path
    /// \param dist the distance along the path sicne the start of the path
    /// \param vel the velocity at this point of the path
    /// \param accel the acceleration at this point of the path
    /// \param jerk the jerk at this point of the path
    /// \param curv the curvature of the path
    /// \param rot the rotation of the robot if its a swerve
    /// \param heading the heading of the robot at this point of the path
    public XeroPathSegment(double time, double x, double y, double dist, double vel, double accel, double jerk, double heading, double curv, double rot) {
        time_ = time ;
        x_ = x ;
        y_ = y ;
        pos_ = dist ;
        vel_ = vel ;
        accel_ = accel ;
        jerk_ = jerk ;
        heading_ = heading ;
        curvature_ = curv ;
        rotation_ = rot ;
    }

    /// \brief create a new path segment
    /// The data for the segment is stored in the data array.  In order, the data in the array must be
    /// time, x, y, distance, velocity, acceleration, jerk, and heading.
    /// \param data the data for the segment.
    public XeroPathSegment(Double[] data) {
        time_ = data[0] ;
        x_ = data[1] ;
        y_ = data[2] ;
        pos_ = data[3] ;
        vel_ = data[4] ;
        accel_ = data[5] ;
        jerk_ = data[6] ;
        heading_ = data[7] ;
    }

    /// \brief returns the time for the path segment
    /// \returns the time for the path segment
    public double getTime() {
        return time_ ;
    }

    /// \brief returns the x postion for the path segment
    /// \returns the x position for the path segment
    public double getX() {
        return x_ ;
    }

    /// \brief returns the y position for the path segment
    /// \returns the y position for the path segment
    public double getY() {
        return y_ ;
    }

    /// \brief returns the position along the path segment
    /// \returns the position along the path segment    
    public double getPosition() {
        return pos_ ;
    }

    /// \brief returns the velocity for the path segment
    /// \returns the velocity for the path segment    
    public double getVelocity() {
        return vel_ ;
    }

    /// \brief returns the acceleration for the path segment
    /// \returns the acceleration for the path segment    
    public double getAccel() {
        return accel_ ;
    }

    /// \brief returns the time for the path segment
    /// \returns the time for the path segment    
    public double getJerk() {
        return jerk_ ;
    }

    /// \brief returns the heading for the path segment
    /// \returns the heading for the path segment      
    public double getHeading() {
        return heading_ ;
    }

    /// \brief returns the curvature for the path segment
    /// \returns the curvature for the path segment      
    public double getCurvature() {
        return curvature_ ;
    }

    /// \brief returns the robot rotation for swerve for the path segment
    /// \returns the robot rotation for swerve for the path segment      
    public double getRotation() {
        return rotation_ ;
    }
}