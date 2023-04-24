package org.xero1425.base.motors ;

import java.util.List ;
import java.util.ArrayList ;

/// \file

/// \brief This class acts like a MotorController but in reality is a group of motors that are mechanically
/// coupled and setup such that all but the first motor follows the first motor.  For the most part, calls to this
/// object are referred to the first motor controller in the group.
public class MotorGroupController extends MotorController
{ 
    // The set of motors that are grouped
    private List<MotorController> motors_ ;

    /// \brief Create a new MotorGroupController
    /// \param name the name of the group
    public MotorGroupController(String name) {
        super(name) ;
        motors_ = new ArrayList<MotorController>() ;
    }

    public int[] getPDPChannels() {
        int [] channels = new int[motors_.size()] ;
        for(int i = 0 ; i < motors_.size() ; i++) {
            channels[i] = motors_.get(i).getPDPChannel() ;
        }

        return channels ;
    }

    /// \brief Return a specific motor from the group.  This is used for debugging issues.
    /// \param index the index of the motor to retreive
    /// \returns a specific motor controller from the group
    public MotorController getMotor(int index) {
        return motors_.get(index) ;
    }

    public void setNeutralDeadband(double value) throws BadMotorRequestException, MotorRequestFailedException {
        for(MotorController ctrl : motors_) {
            ctrl.setNeutralDeadband(value);
        }
    }

    /// \brief Add a new motor to the group
    /// \param ctrl the motor to add to the group
    /// \param leader if true, the leader is inverted
    /// \param inverted if true, the new motor is inverted with respect to the first motor
    public void addFollowerMotor(MotorController ctrl, boolean leader, boolean inverted) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() > 0 && !motors_.get(0).getType().equals(ctrl.getType()))
            throw new BadMotorRequestException(this, "cannot add motor to group with existing motors unless the are the same type") ;

        motors_.add(ctrl) ;

        if (motors_.size() > 1)
            ctrl.follow(motors_.get(0), leader, inverted) ;
    }

    public void addLeaderMotor(MotorController ctrl) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() > 0 && !motors_.get(0).getType().equals(ctrl.getType()))
            throw new BadMotorRequestException(this, "cannot add motor to group with existing motors unless the are the same type") ;

        motors_.add(ctrl) ;
    }    

    /// \brief Return the velocity of the motor when the PID loops is being run in the controller
    /// \retruns the velocity of the motor group.
    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getVelocity() ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller    
    public double getInputVoltage() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getInputVoltage() ;
    }    

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor        
    public double getAppliedVoltage() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getAppliedVoltage() ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller    
    public boolean hasPID(PidType type) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).hasPID(type) ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller       
    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setTarget(target);
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller     
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).setPID(type, p, i, d, f, outmax) ;
    }

    /// \brief Stop the PID loop in the motor controller     
    public void stopPID() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).stopPID() ;
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor      
    public void set(double percent) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).set(percent) ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not     
    public void setInverted(boolean inverted)  throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
            
        motors_.get(0).setInverted(inverted);
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted      
    public boolean isInverted() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
            
        return motors_.get(0).isInverted() ;
    }

    /// \brief Reapplies the inverted status of the motor.  When setInverted() is called, the inverted state of the motor
    /// is stored and this method reapplies that stored state to the motor controller.  This was put into place because some
    /// motors setup to follow other motors lost their inverted state when the robot was disabled and re-enabled.    
    // public void reapplyInverted()  throws BadMotorRequestException, MotorRequestFailedException {
    //     if (motors_.size() == 0)
    //         throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;    
    //     motors_.get(0).reapplyInverted();        
    // }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor      
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        for(MotorController ctrl : motors_)
            ctrl.setNeutralMode(mode);
    }

    /// \brief Set the current motor to follow another motor.  Note a MotorGroupController cannot follow anything else.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader is inverted
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.      
    public void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "a motor group cannot follow other motors") ;
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type        
    public String getType() throws BadMotorRequestException, MotorRequestFailedException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getType() ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position      
    public boolean hasPosition() throws BadMotorRequestException{
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).hasPosition() ;
    }

    /// \brief Returns the position of the motor in motor units.
    public double getPosition() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        return motors_.get(0).getPosition() ;  
    }

    /// \brief Reset the encoder values to zero      
    public void resetEncoder() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        motors_.get(0).resetEncoder();
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given   
    public void setCurrentLimit(double free, double stall) throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
                    
        for(MotorController ctrl : motors_)
            ctrl.setCurrentLimit(free, stall);
    }      

    /// \brief Set the open loop ramp rate for the motor
    /// \param limit the amount of time for the motor to ramp from no power to full power       
    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;
                    
        for(MotorController ctrl : motors_)
            ctrl.setOpenLoopRampRate(limit);
    }   

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller       
    public String getFirmwareVersion() throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        StringBuilder result = new StringBuilder() ;

        for(int i = 0 ; i < motors_.size() ; i++) {
            if (result.length() > 0)
                result.append(",") ;
            
            result.append(motors_.get(i).getFirmwareVersion()) ;
        }

        return result.toString() ;
    }

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values        
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency pos, EncoderUpdateFrequency vel) throws BadMotorRequestException {
        if (motors_.size() == 0)
            throw new BadMotorRequestException(this, "request made to empty MotorGroupController") ;

        int which = 0 ;
        for(MotorController ctrl : motors_)
        {
            if (which == 0)
                ctrl.setEncoderUpdateFrequncy(pos, vel);
            else
                ctrl.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent) ;
        }
    }
} ;
