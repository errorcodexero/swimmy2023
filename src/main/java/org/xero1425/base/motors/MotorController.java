package org.xero1425.base.motors ;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/// \file

/// \brief This class is an abstract base class that defines a contract that all supported motors
/// must meet.  There are specific derived classes for Talon SRX, Victor SPX, Talon Fx, SparkMax, and Romi
/// motor controller.  There are also derived classes where a group of mechanically connected motors
/// are represented by a single MotorController derived object (MotorGroupController).
public abstract class MotorController
{
    // The name of the motor
    private String name_ ;

    // The PDP channel for the motor
    private int pdp_channel_ ;

    /// \brief Property name for property used for motor power in a simulation
    public final static String SimPowerParamName = "Power" ;

    /// \brief Property name for property used for the encoder value in a simulation    
    public final static String SimEncoderParamName = "Encoder" ;

    /// \brief Property name for property used for the stores ticks in a simulation (i.e. motor contains encoder)
    public final static String SimEncoderStoresTicksParamName = "StoresTicks" ;

    /// \brief Property name for property used for to indicate if the power should be inverted in a simulation
    public final static String SimInvertedParamName = "Inverted" ;

     /// \brief Property name for property used for to indicate the neutral mode in a simulation  
    public final static String SimNeutralParamName = "Neutral" ;

    /// \brief The NeutralMode for the motor
    public enum NeutralMode { 
        Coast,              ///< Coast mode
        Brake               ///< Brake mode
    } ;

    /// \brief This enumeration defines how frequencly encoder should be updated
    public enum EncoderUpdateFrequency {
        Frequent,           ///< Encoders should be updated as frequently as possible (once per robot loop)
        Default,            ///< Encoders should be updated at a nominal rate (every few robot loops)
        Infrequent          ///< Encoders are not sample frequently, once every second or two
    } ;

    /// \brief Create a new motor controller
    /// \param name the name of the motor controller
    MotorController(String name) {
        name_ = name ;
        pdp_channel_ = -1 ;
    }

    protected void setPDPChannel(int channel) {
        pdp_channel_ = channel ;
    }

    public int[] getPDPChannels() {
        int [] ret  = new int[1] ;
        ret[0] = pdp_channel_ ;
        return ret;
    }

    public int getPDPChannel() {
        return pdp_channel_ ;
    }

    public boolean hasPDPChannel() {
        return pdp_channel_ != -1 ;
    }

    /// \brief Returns the name of the motor controller
    /// \returns the name of the motor controller
    public String getName() {
        return name_  ;
    }

    /// \brief PidType the type of PID control to run on the motor controller
    public enum PidType {
        None,                       ///< No PID type has been set
        Position,                   ///< Position PID control
        Velocity,                   ///< Velocity PID control
        Magic,                      ///< Motion Magic
    }

    public abstract void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return the velocity of the motor if there is PID control in the motor controller
    public abstract double getVelocity()  throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor
    public abstract void set(double percent)  throws BadMotorRequestException, MotorRequestFailedException ;
    
    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not
    public abstract void setInverted(boolean inverted)  throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted
    public abstract boolean isInverted() throws BadMotorRequestException , MotorRequestFailedException ;    

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public abstract void setNeutralMode(NeutralMode coast) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader is inverted versus normal operation
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.
    public abstract void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type
    public abstract String getType()  throws BadMotorRequestException, MotorRequestFailedException ;

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller
    public abstract double getInputVoltage() throws BadMotorRequestException , MotorRequestFailedException ;
    
    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public abstract boolean hasPID(PidType type) throws BadMotorRequestException , MotorRequestFailedException ;

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller
    public abstract void setTarget(double target) throws BadMotorRequestException , MotorRequestFailedException ;

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller 
    public abstract void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException , MotorRequestFailedException ;

    /// \brief Stop the PID loop in the motor controller
    public abstract void stopPID() throws BadMotorRequestException , MotorRequestFailedException ;

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller
    public abstract String getFirmwareVersion() throws BadMotorRequestException ;

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor
    public abstract double getAppliedVoltage() throws BadMotorRequestException ;

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values
    public abstract void setEncoderUpdateFrequncy(EncoderUpdateFrequency pos, EncoderUpdateFrequency vel) throws BadMotorRequestException ;

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position
    public boolean hasPosition() throws BadMotorRequestException {
        return false ;
    }

    /// \brief Returns the position of the motor in motor units from the motor controller.  If the motor does not
    /// have an attached encoder, an exception is thrown.
    /// \returns the position of the motor in motor units
    public double getPosition() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support getPosition()") ;
    }

    /// \brief Reset the encoder values to zero
    public void resetEncoder() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support resetEncoder()") ;        
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given
    public void setCurrentLimit(double free, double stall) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support setCurrentLimit()") ;        
    }

    /// \brief Set the open loop ramp rate for the motor
    /// \param ramptime the amount of time for the motor to ramp from no power to full power
    public void setOpenLoopRampRate(double ramptime) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support setOpenLoopRampRate()") ;    
    }

    /// \brief Returns the number of ticks per revolution for the motor if it has an embedded encoder
    /// \returns the number of ticks per revolution for the motor if it has an embedded encoder
    public double TicksPerRevolution() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "this motor does not support an embedded encoder")  ;
    }

    public abstract TalonFX getTalonFX() throws BadMotorRequestException ;
}
