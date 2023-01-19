package org.xero1425.misc ;

import java.util.List ;
import java.util.ArrayList ;

/// \file

/// \brief This class is used to measure the speed, velocity, and acceleration of a physical quantity.
/// This quantity can be either a distance or an angle.  For distances, the position input is expected to be
/// linear.  For angles, the position input is expected to be between -180 and 180 degrees.  This class tracks a
/// number of samples which is configurable and computes the outputs based on the set of samples.  Note, the more
/// samples, the more accurate the values are likely to be, especially in a noisy environment, but the bigger the
/// latency.
public class Speedometer
{
    //
    // If true, we are measuring an angle
    //
    private boolean angle_ ;

    //
    // The maximum number of samples to keep
    //
    private int max_samples_ ;

    //
    // The set of distances stored, up to max_samples_
    //
    private List<Double> distances_ ;

    //
    // The set of calculated velocities stored, up to max_samples_
    //
    private List<Double> velocities_ ;

    //
    // The set of times stored, up to max_samples_
    //
    private List<Double> times_ ;

    //
    // The acceleration of the system
    //
    private double accel_ ;

    //
    // The name of the speedometer
    //
    private String name_ ;

    /// \brief create a new speedometer
    /// \param name the name of the speedometer
    /// \param samples the number of samples to keep
    /// \param angle if true, we are measuring an angle
    public Speedometer(String name, int samples, boolean angle) {
        angle_ = angle ;
        max_samples_ = samples ;
        distances_ = new ArrayList<Double>() ;
        velocities_ = new ArrayList<Double>() ;
        times_ = new ArrayList<Double>() ;
        accel_ = 0.0 ;
        name_ = name ;
    }

    /// \brief returns the name of the speedometer
    /// \returns the name of the speedomoeter
    public String getName() {
        return name_ ;
    }

    /// \brief update the speedometer with a new sample
    /// \param dtime the delta time since the last sample
    /// \param pos the new position of the quantity being measured
    public void update(double dtime, double pos) {
        double vel ;

        if (dtime > 1e-4) {
            times_.add(dtime) ;
            if (times_.size() > max_samples_)
                times_.remove(0) ;

            distances_.add(pos) ;
            if (distances_.size() > max_samples_)
                distances_.remove(0) ;

            double total = 0.0 ;
            for(int i = 1 ; i < times_.size() ; i++)
                total += times_.get(i) ;

            if (angle_)
                vel = XeroMath.normalizeAngleDegrees(getDistance() - getOldestDistance()) / total ;
            else
                vel = (getDistance() - getOldestDistance()) / total ;

            velocities_.add(vel) ;
            if (velocities_.size() > max_samples_)
                velocities_.remove(0) ;

            accel_ = (getVelocity() - getOldestVelocity()) / total ;
        }
    }    

    /// \brief return the distance traveled.
    /// This method returns the latest position sample provided via the update method.  If no
    /// samples have been provided, this method returns 0.0.
    /// \returns the distance traveled
    public double getDistance() {
        if (distances_.size() == 0)
            return 0.0 ;

        return distances_.get(distances_.size() - 1) ;
    }

    /// \brief return the computed velocity of the system
    /// This method returns the latest velocity based on the samples provided to date.  If no
    /// samples have been provided, this method returns 0.0.
    /// \returns the velocity of the system
    public double getVelocity() {
        if (velocities_.size() == 0)
            return 0.0 ;

        return velocities_.get(velocities_.size() - 1) ;        
    }

    /// \brief return the computed acceleration of the system
    /// This method returns the latest acceleration based on the samples provided to date.  If zero or one
    /// samples have been provided, this method returns 0.0.
    /// \returns the acceleration of the system
    public double getAcceleration() {
        return accel_ ;
    }

    /// \brief return the average of the distance samples stored
    /// \returns the average of the distance samples stored
    public double getAverage() {
        double total = 0 ;
        for(int i = 0 ; i < distances_.size() ; i++)
            total += distances_.get(i) ;

        return total / distances_.size() ;
    }

    /// \returns the oldest distance
    private double getOldestDistance()  {
        return distances_.get(0) ;
    }

    /// \returns the oldest velocity
    private double getOldestVelocity() {
        return velocities_.get(0) ;
    }
}