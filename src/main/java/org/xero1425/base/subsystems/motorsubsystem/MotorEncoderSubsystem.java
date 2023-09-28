package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.PidType;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.Speedometer;

/// \file

/// \brief A subsystem that includes a single motor, or group of motors mechanically coupled and an encoder for closed loop control
public class MotorEncoderSubsystem extends MotorSubsystem
{
    // The speedometer measuring the speed of the motor output
    private Speedometer speedometer_ ;

    // The encoder attached to the motor output
    private XeroEncoder encoder_ ;

    // If true, the measured output is angular
    private boolean angular_ ;

    // If true, use the velocity from the motor controller
    private boolean use_ctrl_velocity_ ;

    private double max_value_ ;
    private double min_value_ ;

    private boolean dump_currents_ ;


    /// \brief Create the subsystem
    /// \param parent the owning subsystem
    /// \param name the name of this subsystem
    /// \param angle if true, the measured output is angular
    /// \param velconv the velocity conversion if using the velocity from the motor controller
    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angle, boolean usectrlvel) throws Exception {
        super(parent, name) ;

        speedometer_ = new Speedometer(name, 2, angle) ;
        angular_ = angle ;

        use_ctrl_velocity_ = false ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angle, getMotorController()) ;

        if (usectrlvel && !XeroRobot.isSimulation()) {
            if (!encoder_.isMotorEncoder()) {
                String msg = "motor '" + name + "', you can only use the velocity from the motor if the encoder is in the motor" ;
                throw new Exception(msg) ;
            }

            use_ctrl_velocity_ = true ;
        }

        if (isSettingDefined("maxpos"))
            max_value_ = getSettingsValue("maxpos").getDouble() ;
        else
            max_value_ = Double.MAX_VALUE ;

        if (isSettingDefined("minpos"))
            min_value_ = getSettingsValue("minpos").getDouble() ;
        else
            min_value_ = -Double.MAX_VALUE ;

        dump_currents_ = false ;
    }


    /// \brief Create the subsystem
    /// \param parent the owning subsystem
    /// \param name the name of this subsystem
    /// \param angle if true, the measured output is angular
    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angle) throws Exception {
        super(parent, name) ;

        speedometer_ = new Speedometer(name, 2, angle) ;
        angular_ = angle ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angle, getMotorController()) ;

        use_ctrl_velocity_ = false ;

        if (isSettingDefined("maxpos"))
            max_value_ = getSettingsValue("maxpos").getDouble() ;
        else
            max_value_ = Double.MAX_VALUE ;

        if (isSettingDefined("minpos"))
            min_value_ = getSettingsValue("minpos").getDouble() ;
        else
            min_value_ = -Double.MAX_VALUE ;

        dump_currents_ = false ;            
    }    

    /// \brief Create the subsystem
    /// \param parent the owning subsystem
    /// \param name the name of this subsystem
    /// \param angle if true, the measured output is angular
    /// \param samples the number of samples to average for output position and velocity
    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angle, int samples, boolean usectrlvel) throws Exception {
        super(parent, name) ;

        speedometer_ = new Speedometer(name, samples, angle) ;
        angular_ = angle ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angle, getMotorController()) ;

        if (usectrlvel && !XeroRobot.isSimulation()) {
            use_ctrl_velocity_ = true ;
        }

        if (isSettingDefined("maxpos"))
            max_value_ = getSettingsValue("maxpos").getDouble() ;
        else
            max_value_ = Double.MAX_VALUE ;

        if (isSettingDefined("minpos"))
            min_value_ = getSettingsValue("minpos").getDouble() ;
        else
            min_value_ = -Double.MAX_VALUE ;        
     
        dump_currents_ = false ;            
    }

    /// \brief Create the subsystem
    /// \param parent the owning subsystem
    /// \param name the name of this subsystem
    /// \param angle if true, the measured output is angular
    /// \param samples the number of samples to average for output position and velocity
    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angle, int samples) throws Exception {
        super(parent, name) ;

        speedometer_ = new Speedometer(name, samples, angle) ;
        angular_ = angle ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angle, getMotorController()) ;

        use_ctrl_velocity_ = false ;
        dump_currents_ = false ;        
    }

    public XeroEncoder getEncoder() {
        return encoder_ ;
    }

    public void setDumpCurrents(boolean b) {
        dump_currents_ = b ;
    }

    public String getUnits() {
        return encoder_.getUnits();
    }
    
    public double getMaxPos() {
        return max_value_ ;
    }

    public double getMinPos() {
        return min_value_ ;
    }

    /// \brief Returns true if the motor has a hardware PID loop in the motor controller
    /// \returns true if the motor has a hardware PID loop in the motor controller
    public boolean hasHWPID(PidType type) {
        boolean ret = false ;

        try {
            ret = getMotorController().hasPID(type) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret;
    }

    /// \brief Returns true if measuring an angular quantity
    /// \returns true if measuring an angular quantity
    public boolean isAngular() {
        return angular_ ;
    }

    /// \brief Returns the position of the motor output, as measured by the speedometer
    /// \returns the position of the motor output, as measured by the speedometer
    public double getPosition() {
        return speedometer_.getDistance() ;
    }
    
    public double velocityToController(double vel) {
        return encoder_.mapVelocityToMotor(vel) ;
    }

    /// \brief Returns the velocity of the motor output, as measured by the speedometer
    /// \returns the velocity of the motor output, as measured by the speedometer    
    public double getVelocity() {
        double ret = 0.0 ;

        if (use_ctrl_velocity_) {
            try {
                ret = encoder_.mapMotorToVelocity(getMotorController().getVelocity()) ;
            } catch (BadMotorRequestException | MotorRequestFailedException e) {
                ret = 0.0 ;
            }
        }
        else {
            ret = speedometer_.getVelocity() ;
        }

        return ret ;
    }

    /// \brief Returns the acceleration of the motor output, as measured by the speedometer
    /// \returns the acceleration of the motor output, as measured by the speedometer     
    public double getAcceleration() {
        double ret = 0.0 ;

        if (!use_ctrl_velocity_) {
            ret = speedometer_.getAcceleration() ;
        }

        return ret ;
    }

    /// \brief Calibrates the motor encoder with the given position
    /// \param pos the current real world position of the motor output
    public void calibrate(double pos) {
        encoder_.calibrate(pos) ;
    }

    @Override
    public void postHWInit() throws Exception {
        super.postHWInit();
        encoder_.reset() ;
    }

    /// \brief Reset the motor and attacd encoder.  This will reset the encoder value to 
    /// zero and set the motor power to off.
    public void reset() {
        super.reset() ;
    }

    /// \brief Returns the encoder raw count
    /// \returns the encoder raw count
    public double getEncoderRawCount() {
        return encoder_.getRawCount() ;
    }

    public double getTotalCurrent() {
        double total = 0.0 ;
        int [] channels = getMotorController().getPDPChannels() ;

        for(int i = 0 ; i < channels.length ; i++) {
            total += getRobot().getCurrent(channels[i]);
        }

        return total ;
    }

    /// \brief Called once per robot loop by the Xero Framework to compute the position, velocity, and
    /// acceleration of the motor in real world units.  It also displays the position and velocity on the
    /// SmartDashboard if verbose output is enabled for this subsystem.
    @Override
    public void computeMyState() throws Exception {
        
        super.computeMyState();

        double pos = encoder_.getPosition() ;

        speedometer_.update(getRobot().getDeltaTime(), pos) ;

        MessageLogger logger = getRobot().getMessageLogger()  ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add(getName()) ;
        logger.add("power", getPower()) ;
        logger.add("pos", pos, "%.0f") ;
        logger.add("velocity", speedometer_.getVelocity(), "%.0f");
        logger.add("accel", speedometer_.getAcceleration(), "%.0f") ;
        logger.add("ticks", getEncoderRawCount()) ;
        logger.endMessage();

        putDashboard(getName() + "-position", DisplayType.Verbose, pos);
        putDashboard("raw", DisplayType.Verbose, encoder_.getRawCount());

        if (dump_currents_) {
            double current = getTotalCurrent() ;

            logger.startMessage(MessageType.Info) ;
            logger.add(getName()) ;
            logger.add("current", current) ;
            logger.endMessage();
        }
    }
}
