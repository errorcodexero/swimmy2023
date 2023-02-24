package org.xero1425.misc ;

/// \file

/// \brief this class is a solver for the trapezoidal speed profile required to travel a distance
public class TrapezoidalProfile implements IMotionProfile {

    private TrapezoidalProfileConfig config_ ;

    //
    // The time spend accelerating
    //
    private double ta_ ;

    //
    // The time spend cruising at maxv
    //
    private double tc_ ;

    //
    // The time spend decelerating
    //
    private double td_ ;

    //
    // The start velocity for the profile
    //
    private double start_velocity_ ;

    //
    // The end velocity for the profile
    //
    private double end_velocity_ ;

    //
    // The distance covered by the profile
    //
    private double distance_ ;

    //
    // If true, the final output should be negative
    //
    private boolean isneg_ ;

    //
    // The actual max velocity
    //
    private double actual_max_velocity_ ;

    //
    // The type of the actual profile (trapezoid, v, or line)
    //
    private String type_ ;

    /// \brief create the object given the performance characteristics of the movement
    /// \param accel the maximum acceleration
    /// \param decel the maximum deceleration
    /// \param maxv the maximum velocity
    public TrapezoidalProfile(TrapezoidalProfileConfig c) {
        config_ = c ;
    }

    /// \brief create the object reading the performance characteristics from the settings file
    /// The max acceleration is found by appending ":maxa" to the basename.  The maximum deceleration
    /// is found by appending ":maxd" to the basename.  The maximum velocity is found by appending
    /// ":maxv" to the basename.
    /// \param settings the settings file parser
    /// \param name the basename used to look up parameters.
    public TrapezoidalProfile(ISettingsSupplier settings, String name) throws BadParameterTypeException, MissingParameterException {
        double maxa = settings.get(name + ":maxa").getDouble() ;
        double maxd = settings.get(name + ":maxd").getDouble() ;
        double maxv = settings.get(name + ":maxv").getDouble() ;

        config_ = new TrapezoidalProfileConfig(maxa, maxd, maxv);
    }

    /// \brief create a speed profile that covers the distance given, with the start and end velocities as conditions
    /// The acceleration time (if any), cruise time (if any), and deceleration time (if any) are all stored internally.
    /// \param dist the distance the speed profile should cover
    /// \param start_velocity the start velocity of the object
    /// \param end_velocity the end velocity of the object
    public boolean update(double dist, double start_velocity, double end_velocity) {
        start_velocity_ = Math.abs(start_velocity) ;
        end_velocity_ = Math.abs(end_velocity) ;

        isneg_ = (dist < 0) ;
        distance_ = Math.abs(dist) ;

        ta_ = (config_.velocity - start_velocity_) / config_.accel ;

        td_ = (end_velocity_ - config_.velocity) / config_.decel ;

        //distance accelerating
        double da = start_velocity * ta_ + 0.5 * config_.accel * ta_ * ta_ ;
        //distance decelerating
        double dd = config_.velocity * td_ + 0.5 * config_.decel * td_ * td_ ;
        
        tc_ = (distance_ - da - dd) / config_.velocity ;
        type_ = "trapezoid" ;

        if (td_ < 0.0 || da + dd > distance_) {
            //
            // We don't have time to get to the cruising velocity
            //
            double num = (2.0 * distance_ * config_.accel * config_.decel + config_.decel * start_velocity_ * start_velocity_ - config_.accel * end_velocity_ * end_velocity_) / (config_.decel - config_.accel) ;
            boolean decel_only = false ;
            if (num < 0)
                decel_only = true ;
            else
                actual_max_velocity_ = Math.sqrt(num) ;


            if (decel_only || actual_max_velocity_ < start_velocity_) {
                // 
                // Just decelerate down to the end
                //
                ta_ = 0 ;
                tc_ = 0 ;
                td_ = (end_velocity - start_velocity_) / config_.decel ;
                actual_max_velocity_ = start_velocity_ ;
                type_ = "line" ;
            }
            else {
                //
                // Can't get to max velocity but can accelerate some
                // before decelerating
                //
                actual_max_velocity_ = Math.sqrt(num) ;
                ta_ = (actual_max_velocity_ -start_velocity_)/ config_.accel ;
                td_ = (end_velocity_ - actual_max_velocity_) / config_.decel ;
                tc_ = 0 ;
                type_ = "pyramid" ;
            }
        }
        else {
            //
            // Okay, now figure out the crusing time
            //
            actual_max_velocity_ = config_.velocity ;                
            tc_ = (distance_ - da - dd) / config_.velocity ;
        }

        return true ;
    }

    /// \brief return the planned acceleration for the time given relative to the time when update() was called
    /// \param t the time of interest
    /// \returns the acceleration for the time of interest
    public double getAccel(double t) {
        double ret ;

        if (t < 0)
            ret = 0 ;
        else if (t < ta_)
            ret = config_.accel ;
        else if (t < ta_ + tc_)
            ret = 0.0 ;
        else if (t < ta_ + tc_ + td_)
            ret = config_.decel ;
        else
            ret = 0.0 ;

        return isneg_ ? -ret : ret ;
    }

    /// \brief return the planned velocity for the time given relative to the time when update() was called
    /// \param t the time of interest
    /// \returns the velocity for the time of interest
    public double getVelocity(double t) {
        
        double ret ;
        if (t < 0.0) {
            ret = start_velocity_ ;
        }
        else if (t < ta_) {
            ret = start_velocity_ + t * config_.accel ;
        }   
        else if (t < ta_ + tc_) {
            ret = actual_max_velocity_ ;
        }   
        else if (t < ta_ + tc_ + td_) {
            double dt = (t - ta_ - tc_) ;
            ret = actual_max_velocity_ + dt * config_.decel ;
        }
        else {
            ret = end_velocity_ ;
        }

        return isneg_ ? -ret : ret ;
    }

    /// \brief return the planned distance for the time given relative to the time when update() was called
    /// \param t the time of interest
    /// \returns the distance for the time of interest
    public double getDistance(double t) {
        double ret ;

        if (t < 0.0) {
            ret = 0.0 ;
        }
        else if (t < ta_) {
            ret = start_velocity_ * t + 0.5 * t * t * config_.accel ;
        }   
        else if (t < ta_ + tc_) {
            ret = start_velocity_ * ta_ + 0.5 * ta_ * ta_ * config_.accel ;
            ret += (t - ta_) * actual_max_velocity_ ;
        }   
        else if (t < ta_ + tc_ + td_) {
            double dt = t - ta_ - tc_ ;
            ret = start_velocity_ * ta_ + 0.5 * ta_ * ta_ * config_.accel ;
            ret += tc_ * actual_max_velocity_ ;
            ret += actual_max_velocity_ * dt + 0.5 * dt * dt * config_.decel ;
        }
        else {
            ret = distance_ ;
        }

        return isneg_ ? -ret : ret ;
    }

    /// \brief return a human readable string describing the speed profile
    /// \returns a human readable string describing the speed profile
    public String toString() {
        String ret = "[" + type_ ;
        ret += ", sv " + Double.toString(start_velocity_) ;
        ret += ", mv " + Double.toString(actual_max_velocity_) ;
        ret += ", ev " + Double.toString(end_velocity_) ;
        ret += ", ta " + Double.toString(ta_) ;
        ret += ", tc " + Double.toString(tc_) ;
        ret += ", td " + Double.toString(td_) ;                        
        ret += "]" ;

        return ret ;
    }

    /// \brief return the time spent acceleration
    /// \returns the time spent acceleration
    public double getTimeAccel() {
        return ta_ ;
    }

    /// \brief return the time spent cruising
    /// \returns the time spent cruising
    public double getTimeCruise() {
        return tc_ ;
    }

    /// \brief return the time spent deceleration
    /// \returns the time spent deceleration
    public double getTimeDecel() {
        return td_ ;
    }

    /// \brief return the total time for the profile
    /// \returns the total time for the profile
    public double getTotalTime() {
        return ta_ + tc_ + td_ ;
    }

    /// \brief return the maximum velocity in the profile
    /// This might just be maxv provided in the update() call if there is time for the profile to
    /// reach this velocity.  However, if the profile is a "V" shape instead of a trapezoidal shape
    /// then this maximum velocity may be less than maxv.
    /// \returns the maximum velocity in the profile
    public double getActualMaxVelocity() {
        if (isneg_)
            return -actual_max_velocity_ ;

        return actual_max_velocity_ ;
    }

    /// \brief given a distance, return the when that distance will be hit
    /// \returns the time when a specific distance will be hit.
    public double getTimeForDistance(double dist) throws Exception {
        double ret ;
        double sign = isneg_ ? -1.0 : 1.0 ;
        double [] roots ;

        if (isneg_)
            dist = -dist ;

        if (dist < sign * getDistance(ta_)) {
            roots = XeroMath.quadratic(0.5 * config_.accel, start_velocity_, -dist) ;
            ret = XeroMath.pickRoot(roots) ;
        }
        else if (dist < sign * getDistance(ta_ + tc_)) {
            dist -= sign * getDistance(ta_) ;
            ret = ta_ + dist / actual_max_velocity_ ;
        }
        else if (dist < sign * getDistance(ta_ + tc_ + td_)) {
            dist -= sign * getDistance(ta_ + tc_) ;
            roots = XeroMath.quadratic(0.5 * config_.decel, actual_max_velocity_, -dist) ;
            ret = XeroMath.pickRoot(roots) + ta_ + tc_ ;
        }
        else {
            ret = ta_ + tc_ + td_ ;
        }

        return ret ;
    }

    /// \brief return the start velocity for the speed profile
    /// \returns the start velocity for the speed profile    
    public double getStartVelocity() {
        return start_velocity_ ;
    }

    /// \brief return the end velocity for the speed profile
    /// \returns the end velocity for the speed profile     
    public double getEndVelocity() {
        return end_velocity_ ;
    }


}