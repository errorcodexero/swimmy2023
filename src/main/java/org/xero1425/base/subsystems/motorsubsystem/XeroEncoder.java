package org.xero1425.base.subsystems.motorsubsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This class is a generalization of an encoder.  It can represent the encoder that is part of a
/// brushless motor, an quadrature encoder that is connected to two digital inputs on the RoboRio, an encoder
/// connected to an analog input, or an encoder connected to a PWM.  This class can also represent either an
/// absolute encoder or a relative encoder.  This class manages the translation of the encoder value to real
/// world values (e.g. encoder ticks to inches or degrees).
public class XeroEncoder {
    
    // The quadrature encoder if this is a quadrature encoder type
    private Encoder quad_ ;

    // The coefficients of the quadurature encoder equation (Y = MX + B) to conver
    // encoder units to physical units.
    private double quad_m_ ;
    private double quad_b_ ;

    // If the encoder is part of a brushless motor, this is the motor
    private MotorController motor_ ;

    // If the encoder is an analog encoder
    private AnalogInput analog_ ;

    // If the encoder is a PWM based encoder
    private Counter pwm_ ;

    // The mapper that maps units from encoder units to real world units
    private EncoderMapper mapper_ ;

    // The units for the mapped values
    private String units_ ;

    /// \brief Create a new XeroEncoder object
    /// \param robot the robot this encoder is part of, used to get settings and the message logger
    /// \param cname the name of the encoder
    /// \param angular if true, the real world units are angles that range from -180 to 180
    /// \param ctrl if non-null the motor that contains the encoder
    public XeroEncoder(XeroRobot robot, String cname, boolean angular, MotorController ctrl)
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
            BadMotorRequestException {
        createEncoder(robot, cname, ctrl);
    }

    public String getUnits() {
        return units_ ;
    }

    /// \brief get the raw encoder count from the encoder
    /// \returns the raw encoder count from the encoder
    public double getRawCount() {
        double result = 0.0 ;

        try {
            if (motor_ != null)
                result = motor_.getPosition() ;
            else if (quad_ != null)
                result = quad_.get() ;
            else if (analog_ != null)
                result = analog_.getVoltage() ;
            else if (pwm_ != null)
                result = pwm_.getPeriod() ;
        }
        catch(Exception ex) {
            result = 0.0 ;
        }

        return result ;
    }

    public boolean isMotorEncoder() {
        return motor_ != null ;
    }

    public double getB() {
        return quad_b_ ;
    }

    public double getM() {
        return quad_m_ ;
    }

    /// \brief get the real world position for the encoder
    /// \returns the real world position for the encoder
    public double getPosition() {
        double result = 0.0;

        try {
            if (motor_ != null) {
                result = motor_.getPosition() * quad_m_ + quad_b_;
            }
            else if (quad_ != null) {
                result = quad_.get() * quad_m_ + quad_b_ ;
            }
            else if (analog_ != null) {
                result = mapper_.toRobot(analog_.getVoltage()) ;
            }
            else if (pwm_ != null) {
                result = mapper_.toRobot(pwm_.getPeriod()) ;
            }
        } 
        catch (Exception ex) 
        {
            //
            // THis should never happen, but in case it does
            //
            result = 0.0 ;
        }

        return result;
    }

    public double mapMotorToVelocity(double value) {
        double result = 0.0 ;

        if (motor_ != null) {
            result = value * quad_m_ + quad_b_;
        }
        else if (quad_ != null) {
            result = value * quad_m_ + quad_b_ ;
        }
        else if (analog_ != null) {
            result = mapper_.toRobot(value) ;
        }
        else if (pwm_ != null) {
            result = mapper_.toRobot(value) ;
        }

        return result ;
    }

    public double mapVelocityToMotor(double value) {
        double result = 0.0 ;

        if (motor_ != null) {
            result = (value - quad_b_) / quad_m_ ;
        }
        else if (quad_ != null) {
            result = (value - quad_b_) / quad_m_ ;
        }
        else if (analog_ != null) {
            result = mapper_.toEncoder(value) ;
        }
        else if (pwm_ != null) {
            result = mapper_.toEncoder(value) ;
        }

        return result ;
    }

    /// \brief reset the encoder values to zero at the current position
    /// If the encoder does not support reset (absolute analog encoder), 
    /// this method does nothing.
    public void reset() {
        try {
            if (motor_ != null)
                motor_.resetEncoder();
            else if (quad_ != null)
                quad_.reset() ;
        }
        catch(Exception ex) {

        }
    }

    /// \brief Calibrate the encoder to the current real world position.  This
    /// method resets the encoder count to zero and sets the B term in the mapping
    /// function Y = MX + B to the given position.
    public void calibrate(double pos) {
        if (quad_ != null) {
            quad_.reset() ;
        }
        else if (motor_ != null)
        {
            try {
                motor_.resetEncoder();
            }
            catch(Exception ex) {                
            }
        }
        else if (analog_ != null) {
            //
            // This is an analog absolute encoder.
            //
            mapper_.calibrate(pos, analog_.getVoltage());
        }

        quad_b_ = pos ;
    }

    private void createEncoder(XeroRobot robot, String cname, MotorController ctrl)
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
            BadMotorRequestException {

        ISettingsSupplier settings = robot.getSettingsSupplier() ;                
        String type = settings.get(cname + ":type").getString() ;

        if (type.equals("motor")) {
            createMotorEncoder(robot, cname, ctrl) ;
        }
        else if (type.equals("quad")) {
            createQuadEncoder(robot, cname);
        }
        else if (type.equals("analog")) {
            createAnalogEncoder(robot, cname);
        }
        else if (type.equals("pwm")) {
            createPWMEncoder(robot, cname);
        }
        else {
            throw new EncoderConfigException("motor '" + cname + " - unknown encoder type '" + type + "' - expected 'motor' or 'analog' or 'quad' or 'pwm'") ;
        }

        if (pwm_ == null && analog_ == null && quad_ == null && motor_ == null)
            throw new EncoderConfigException("motor '" + cname + "' - must define a QUAD, PWM, MOTOR, or ANALOG encoder");
    }

    private void createMotorEncoder(XeroRobot robot, String cname, MotorController ctrl)
                throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
                BadMotorRequestException {

        ISettingsSupplier settings = robot.getSettingsSupplier() ;                    

        motor_ = ctrl ;
        if (!motor_.hasPosition())
            throw new EncoderConfigException("motor '" + cname + "' - motor does not have internal encoder");
    
        quad_m_ = settings.get(cname + ":m").getDouble() ;
        quad_b_ = settings.get(cname + ":b").getDouble() ;
        units_ = settings.get(cname + ":units").getString() ;
    }

    private void createQuadEncoder(XeroRobot robot, String cname)
            throws BadParameterTypeException, MissingParameterException, EncoderConfigException,
            BadMotorRequestException {

        ISettingsSupplier settings = robot.getSettingsSupplier() ;

        int i1 = settings.get(cname + ":dinput1").getInteger() ;
        int i2 = settings.get(cname + ":dinput2").getInteger() ;        

        quad_ = new Encoder(i1, i2) ;
        quad_m_ = settings.get(cname + ":m").getDouble() ;
        quad_b_ = settings.get(cname + ":b").getDouble() ;
        units_ = settings.get(cname + ":units").getString() ;
    }

    private void createAnalogEncoder(XeroRobot robot, String cname)
            throws BadParameterTypeException, MissingParameterException {
        ISettingsSupplier settings = robot.getSettingsSupplier() ;

        int a = settings.get(cname+":ainput").getInteger() ;
        analog_ = new AnalogInput(a) ;

        double rmin, rmax ;
        double emin, emax ;
        double rc, ec ;

        rmin = settings.get(cname + ":rmin").getDouble() ;
        rmax = settings.get(cname + ":rmax").getDouble() ;
        emin = settings.get(cname + ":emin").getDouble() ;
        emax = settings.get(cname + ":emax").getDouble() ;
        rc = settings.get(cname + ":rc").getDouble() ;
        ec = settings.get(cname + ":ec").getDouble() ;
        units_ = settings.get(cname + ":units").getString() ;

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec) ;
    }

    private void createPWMEncoder(XeroRobot robot, String cname)
            throws BadParameterTypeException, MissingParameterException {
                
        ISettingsSupplier settings = robot.getSettingsSupplier() ;
                
        int a = settings.get(cname + ":dinput").getInteger() ;
        pwm_ = new Counter(a) ;
        pwm_.setSemiPeriodMode(true);

        double rmin, rmax ;
        double emin, emax ;
        double rc, ec ;

        rmin = settings.get(cname + ":rmin").getDouble() ;
        rmax = settings.get(cname + ":rmax").getDouble() ;
        emin = settings.get(cname + ":emin").getDouble() ;
        emax = settings.get(cname + ":emax").getDouble() ;
        rc = settings.get(cname + ":rc").getDouble() ;
        ec = settings.get(cname + ":ec").getDouble() ;
        units_ = settings.get(cname + ":units").getString() ;

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec) ;
    }
} ;
