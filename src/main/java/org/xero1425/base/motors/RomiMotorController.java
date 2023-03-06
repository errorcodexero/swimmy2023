package org.xero1425.base.motors;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/// \file

/// \brief This class is MotorController class that supports the Romi motor controller.   This class is a 
/// wrapper for the Spark class that provides the interface that meets the requirements of the 
/// MotorController base class.  The Romi robot is always run as a simulation, so under the hood this Spark
/// motor controller, when simulated, is monitored and its output power sent to the Romi robot over the 
/// WI-FI link.
public class RomiMotorController extends MotorController {

    private Spark motor_ ;
    private boolean inverted_ ;

    /// \brief Create a new Romi Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller    
    public RomiMotorController(String name, int index) {
        super(name) ;

        motor_ = new Spark(index) ;
    }

    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        return 0.0 ;
    }

    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException {
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor        
    public void set(double percent) throws BadMotorRequestException {
        motor_.set(percent) ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not       
    public void setInverted(boolean inverted)  throws BadMotorRequestException {
        inverted_ = inverted ;
        motor_.setInverted(inverted);
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted     
    public boolean isInverted() throws BadMotorRequestException {
        return inverted_ ;
    }

    /// \brief Reapplies the inverted status of the motor.  When setInverted() is called, the inverted state of the motor
    /// is stored and this method reapplies that stored state to the motor controller.  This was put into place because some
    /// motors setup to follow other motors lost their inverted state when the robot was disabled and re-enabled.      
    // public void reapplyInverted() throws BadMotorRequestException {
    //     motor_.setInverted(inverted_);
    // }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor 
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException {
        // Ignore neutral mode on Romi
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type        
    public String getType()  throws BadMotorRequestException {
        return "Romi" ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller     
    public double getInputVoltage() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the getInputVoltage() capability") ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor       
    public double getAppliedVoltage() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the getAppliedVoltage() capability") ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller.  Note
    /// always returns false for the Romi motor controller
    /// \returns true if the motor controller supports PID loops on the controller    
    public boolean hasPID() throws BadMotorRequestException {
        return false ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller
    public void setTarget(double target) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the PID capability") ;
    }

    /// \brief Stop the PID loop in the motor controller     
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller    
    public void stopPID() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the PID capability") ;
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller.  Note, this has not been fully
    /// implemented or tested for the SparkMax motor controller.  Note, always throws an exception.
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller       
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller    
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the PID capability") ;        
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader is inverted
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws BadMotorRequestException since the Romi motor controller does not support following
    public void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "the 'Romi' motor does not support the follow() capability") ;
    }

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller     
    public String getFirmwareVersion() throws BadMotorRequestException {
        return "?.?" ;
    }
    
    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values     
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency pos, EncoderUpdateFrequency vel) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "this motor does not contains integrated encoders") ;
    }
}
