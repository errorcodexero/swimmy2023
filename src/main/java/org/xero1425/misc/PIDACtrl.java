package org.xero1425.misc;

/// \file

/// \brief a PD controller that uses both a velocity and acceleration feed forward term.  
/// This controller is usually used to follow a motion plan that provides acceleration, 
/// velocity, and position at regular time steps along a plan.  Both the path following
/// in the tank drive for the drivebase and the TrapezoidalProfile provide outputs that
/// are suitable as inputs to this controller.
public class PIDACtrl
{
    //
    // The acceleration feed forward constant
    //
    private double ka_ ;

    //
    // The velocity feed forward constant
    //
    private double kv_ ;

    //
    // The P constant for P control
    //
    private double kp_ ;

    //
    // The D constant for D control
    //
    private double kd_ ;

    //
    // if true, we are managing an angle quantity
    //
    private boolean angle_ ;

    //
    // The last error value calculated
    //
    private double last_error_ ;

    //
    // The portion of the output due to V feed forward
    //
    private double vpart_ ;

    //
    // The portion of the output due to A feed forward
    //
    private double apart_ ;

    //
    // The portion of the output due to the P term
    //
    private double ppart_ ;

    //
    // The portion of the output due to the D term
    //
    private double dpart_ ;

    /// \brief create a new object by reading parameters from the settings parser.
    /// The kv parameter is found by looking up the basename + ":kv".  The ka parameters is
    /// found by looking up the basename + ":ka".  The kp parameter is found by looking up
    /// the basename + ":kp".  The kd parameter is found by looking up the basename + ":kd".
    /// \param settings the settings parser
    /// \param name the basename to use to extract params from the settings parser
    /// \param angle if true it is managing an angle between =180 and +180
    public PIDACtrl(ISettingsSupplier settings, String name, boolean angle) 
                    throws BadParameterTypeException, MissingParameterException {
        kv_ = settings.get(name + ":kv").getDouble() ;
        ka_ = settings.get(name + ":ka").getDouble() ;
        kp_ = settings.get(name + ":kp").getDouble() ;
        kd_ = settings.get(name + ":kd").getDouble() ;
        angle_ = angle ;
    }

    /// \brief create a new object
    /// \param kv the kv value for the controller
    /// \param ka the ka value for the controller
    /// \param kp the kp value for the controller
    /// \param kd the kd value for the controller
    /// \param angle if true we are controlling an angle quantityt between -180 and +180
    public PIDACtrl(double kv, double ka, double kp, double kd, boolean angle) {
        kv_ = kv ;
        ka_ = ka ;
        kp_ = kp ;
        kd_ = kd ;
        angle_ = angle ;
    }

    /// \brief returns the output value controller
    /// \param a the acceleration at this point of the motion plan
    /// \param v the velocity at this point of the motion plan
    /// \param dtarget the target position at this point of the motion plan
    /// \param dactual the actual position at this point of the motion plan
    /// \param dt the delta time since the last time this was called
    /// \returns the output value for the controller
    public double getOutput(double a, double v, double dtarget, double dactual, double dt) {
        double current_error ;
            
        if (angle_)
            current_error = XeroMath.normalizeAngleDegrees(dtarget - dactual) ;
        else
            current_error = dtarget - dactual ;

        vpart_ = v * kv_ ;
        apart_ = a * ka_ ;
        ppart_ = current_error * kp_ ;
        dpart_ = ((current_error - last_error_) / dt -v) * kd_ ;

        double output = vpart_ + apart_ + ppart_ + dpart_ ;
        last_error_ = current_error ;
        return output ;
    }

    /// \brief returns the V portion of the output value
    /// \returns the V portion of the output value
    public double getVPart() {
        return vpart_ ;
    }

    /// \brief returns the A portion of the output value
    /// \returns the A portion of the output value
    public double getAPart() {
        return apart_ ;
    }

    /// \brief returns the P portion of the output value
    /// \returns the P portion of the output value
    public double getPPart() {
        return ppart_ ;
    }

    /// \brief returns the D portion of the output value
    /// \returns the D portion of the output value
    public double getDPart() {
        return dpart_ ;
    }

    /// \brief returns the last error value
    /// \returns the last error value
    public double getLastError() {
        return last_error_ ;
    }    

} ;
