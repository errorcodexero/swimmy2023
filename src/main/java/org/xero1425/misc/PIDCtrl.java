package org.xero1425.misc ;

/// \file

/// \brief This class implements a classic PIDF controller
/// More information about this can be found at https://www.xerosw.org/dokuwiki/doku.php?id=software:followers
///
public class PIDCtrl
{
    //
    // The PIDP constants
    //
    private double kp_ ;
    private double ki_ ;
    private double kd_ ;
    private double kf_ ;

    // The max and min values output
    private double kmin_ ;
    private double kmax_;

    // The max accumulated value for the I term
    private double kimax_;

    // If true, we are managing an angle
    private boolean is_angle_;

    // If true, we have a last error value
    private boolean has_last_error_;

    // The last error
    private double last_error_;

    // The accumulated I term
    private double integral_;

    private double dout_ ;
    private double iout_ ;
    private double pout_ ;
    private double fout_ ;

    /// \brief create a new PIDCtrl object with all parameters set to zero
    /// \param isangle if true this PIDCtrl object manages an angular quantity
    public PIDCtrl(boolean isangle) {
        kp_ = 0 ;
        ki_ = 0 ;
        kd_ = 0 ;
        kf_ = 0 ;
        kmin_ = 0 ;
        kmax_ = 0 ;
        kimax_ = 0 ;

        is_angle_ = isangle ;
    }

    /// \brief create a new object by taking double-inputs.
    /// Enter all the correct information for the variables
    /// \param p the proportional constant
    /// \param i the integral constant
    /// \param d the derivative constant
    /// \param f the feedforward constant
    /// \param minout the minimum output
    /// \param maxout the maximum output
    /// \param maxint the maximum integral
    /// \param isangle if true it is managing an angle between =180 and +180
    public PIDCtrl(double p, double i, double d, double f, double minout, double maxout, double maxint, boolean isangle) {
        kp_ = p ;
        ki_ = i ;
        kd_ = d ;
        kf_ = f ;
        kmin_ = minout ;
        kmax_ = maxout ;
        kimax_ = maxint ;

        is_angle_ = isangle ;
    }

    /// \brief create a new object by reading parameters from the settings parser.
    /// The kv parameter is found by looking up the basename + ":kv".  The ka parameters is
    /// found by looking up the basename + ":ka".  The kp parameter is found by looking up
    /// the basename + ":kp".  The kd parameter is found by looking up the basename + ":kd".
    /// \param settings the settings parser
    /// \param name the basename to use to extract params from the settings parser
    /// \param isangle if true it is managing an angle between =180 and +180
    public PIDCtrl(ISettingsSupplier settings, String name, boolean isangle) throws MissingParameterException, BadParameterTypeException {
        init(settings, name) ;
        is_angle_ = isangle ;
    }

    public void setP(double p) {
        kp_ = p ;
    }

    /// \brief create a new object by reading parameters from the settings parser.
    /// The kv parameter is found by looking up the basename + ":kv".  The ka parameters is
    /// found by looking up the basename + ":ka".  The kp parameter is found by looking up
    /// the basename + ":kp".  The kd parameter is found by looking up the basename + ":kd".
    /// \param settings the settings parser
    /// \param name the basename to use to extract params from the settings parser
    public void init(ISettingsSupplier settings, String name)  throws MissingParameterException, BadParameterTypeException {
        kp_ = settings.get(name + ":kp").getDouble() ;
        ki_ = settings.get(name + ":ki").getDouble() ;
        kd_ = settings.get(name + ":kd").getDouble() ;
        kf_ = settings.get(name + ":kf").getDouble() ;
        kmin_ = settings.get(name + ":min").getDouble() ;
        kmax_ = settings.get(name + ":max").getDouble() ;
        kimax_ = settings.get(name + ":imax").getDouble() ;                                        
    }

    /// \brief return the P component of the PID calculation
    /// \returns the P component of the PID calculation
    public double getPComponent() {
        return pout_ ;
    }

    /// \brief return the I component of the PID calculation
    /// \returns the I component of the PID calculation
    public double getIComponent() {
        return iout_ ;
    }

    /// \brief return the D component of the PID calculation
    /// \returns the D component of the PID calculation    
    public double getDComponent() {
        return dout_ ;
    }

    /// \brief return the F component of the PID calculation
    /// \returns the F component of the PID calculation    
    public double getFComponent() {
        return fout_ ;
    }

    /// \brief get the output by using variables & performing the PID calculations
    /// \param target the target position
    /// \param current the current position
    /// \param dt the difference in time (delta time) since the last robot loop (should be 20 milliseconds)
    /// \returns the output applied to motors/etc. after performing calculations
    public double getOutput(double target, double current, double dt) {
        double error = calcError(target, current) ;
        double derivative = 0;

        pout_ = kp_ * error;

        // dt is difference in time (telta time)
        // "if" statement takes into account whether 1 robot loop has passed since program started
        if (has_last_error_) {
            // takes derivative based on definition of derivative
            derivative = (error - last_error_) / dt ;
        }
        
        // assigns last_error_ to current error
        last_error_ = error;
        has_last_error_ = true;

        // output for derivative * D-constant calculated
        dout_ = kd_ * derivative;
        
        // calculates the integral value by taking a summation of error * difference in time
        integral_ += error * dt ;
        
        // check if integral term is too large small
        if (integral_ > kimax_)
            integral_ = kimax_ ;
        else if (integral_ < -kimax_)
            integral_ = -kimax_ ;
        
        // output fot integral * I-constant calculated 
        iout_ = ki_ * integral_;
        fout_ = kf_ * target ;
        
        // output sum of proportional, integral, and derivative calculations
        // add the feedforward term * target
        double output = pout_ + iout_ + dout_ + fout_ ;
    
        // make sure output isn't too big or small
        // if it is, assign it to the min/max outputs
        if (output <= kmin_)
            output = kmin_ ;
        if (output >= kmax_)
            output = kmax_ ;
        
        return output;
    }

    /// \brief resets "has_last_error_" to default and sets the integral summation back to 0
    public void reset() {
        has_last_error_ = false ;
        integral_ = 0.0 ;
    }
    
    /// \brief gets the error between current and target position
    /// \param target the target position
    /// \param current the current position
    /// \returns the error
    private double calcError(double target, double current) {
        double error ;
        
        // check if target is an angle, if so use a math function to get output in angle degrees between +-180
        // else give a "normal" answer of just (target - current)
        if (is_angle_)
            error = XeroMath.normalizeAngleDegrees(target - current) ;
        else
            error = target - current ;
    
        return error ;        
    }
}