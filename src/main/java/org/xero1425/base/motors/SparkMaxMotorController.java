package org.xero1425.base.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.REVLibError;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \file

/// \brief This class is MotorController class that supports the SparkMax motor controller.   This class is a 
/// wrapper for the CANSparkMax class that provides the interface that meets the requirements of the 
/// MotorController base class.  This class supports both brushless and brushed motors.
public class SparkMaxMotorController extends MotorController
{
    /// \brief A constant that gives the number of ticks per revolution for brushless motors
    private final static int TicksPerRevolutionValue = 42 ;
    private final static double SecondsPerMinuts = 60.0 ;
    private final static double T100msPerSecond = 10.0 ;

    private static double RPM2TicksPer100MS = TicksPerRevolutionValue / SecondsPerMinuts / T100msPerSecond ;

    private CANSparkMax controller_ ;
    private RelativeEncoder encoder_ ;
    private boolean inverted_ ;
    private boolean brushless_ ;
    private SparkMaxPIDController pid_ ;
    private PidType ptype_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimDouble sim_encoder_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    /// \brief The device name in simulation for a brushed motor
    public final static String SimDeviceNameBrushed = "SparkMaxBrushed" ;

    /// \brief The device name in simulation for a brushless motor    
    public final static String SimDeviceNameBrushless = "SparkMaxBrushless" ;



    /// \brief Create a new SparkMax Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller    
    /// \param brushless if true, the motor is a brushless motgor
    /// \param if true, this device has followers
    public SparkMaxMotorController(String name, int index, boolean brushless, boolean leader) throws MotorRequestFailedException {
        super(name, index) ;

        inverted_ = false ;
        brushless_ = brushless ;
        pid_ = null ;
        ptype_ = PidType.None ;

        if (RobotBase.isSimulation()) {
            boolean usesticks = false ;

            if (brushless) {
                sim_ = SimDevice.create(SimDeviceNameBrushless, index) ;
                usesticks = true ;
            }
            else
            {
                sim_ = SimDevice.create(SimDeviceNameBrushed, index) ;
                usesticks = false ;
            }

            sim_power_ = sim_.createDouble(MotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_encoder_ = sim_.createDouble(MotorController.SimEncoderParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(MotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(MotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;  
            sim_.createBoolean(MotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, usesticks) ;        
        }
        else {
            REVLibError code ;

            if (brushless)
            {
                controller_ = new CANSparkMax(index, CANSparkMax.MotorType.kBrushless) ;
            }
            else
            {
                controller_ = new CANSparkMax(index, CANSparkMax.MotorType.kBrushed) ;
            }

            try {
                controller_.restoreFactoryDefaults() ;
                code = controller_.enableVoltageCompensation(11.0) ;
                if (code != REVLibError.kOk) {
                    throw new MotorRequestFailedException(this, "enableVoltageCompensation() failed during initialization", code) ;
                }
            }
            catch(Exception ex) {
            }

            if (leader) {
                code = controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100) ;
                if (code != REVLibError.kOk) {
                    throw new MotorRequestFailedException(this, "Failed to set periodic status frame 0 rate", code) ;
                }
            }
            else {
                code = controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20) ;                
                // if (code != REVLibError.kOk) {
                //     throw new MotorRequestFailedException(this, "Failed to set periodic status frame 0 rate", code) ;
                // }
            }

            code = controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000) ;
            if (code != REVLibError.kOk) {
                throw new MotorRequestFailedException(this, "Failed to set periodic status frame 0 rate", code) ;
            }        
            
            encoder_ = controller_.getEncoder() ;
        }
    }

    public void setNeutralDeadband(double value) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "not supported") ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller    
    public double getInputVoltage() throws BadMotorRequestException {
        if (RobotBase.isSimulation())
            return 12.0 ;

        return controller_.getBusVoltage() ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor      
    public double getAppliedVoltage() throws BadMotorRequestException {
        if (RobotBase.isSimulation())
            return 12.0 ;

        return controller_.getAppliedOutput() ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID(PidType type) throws BadMotorRequestException {
        if (type == PidType.Magic) {
            return false;
        }
        return true ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller     
    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        REVLibError code = REVLibError.kOk ;

        if (pid_ != null) {
            if (ptype_ == PidType.Position)
                code = pid_.setReference(target, CANSparkMax.ControlType.kPosition) ;
            else if (ptype_ == PidType.Velocity)
                code = pid_.setReference(target, CANSparkMax.ControlType.kVelocity) ;
            
            if (code != REVLibError.kOk)
                throw new MotorRequestFailedException(this, "setReference() failed during setTarget() call", code) ;
        }
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller.  Note, this has not been fully
    /// implemented or tested for the SparkMax motor controller.
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller     
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {
        REVLibError code = REVLibError.kOk ;

        if (pid_ == null)
            pid_ = controller_.getPIDController() ;

        code = pid_.setP(p) ;
        if (code != REVLibError.kOk)
            throw new MotorRequestFailedException(this, "setP() failed during setPID() call", code) ;

        code = pid_.setI(i) ;
        if (code != REVLibError.kOk)
            throw new MotorRequestFailedException(this, "setI() failed during setPID() call", code) ;        

        code = pid_.setD(d) ;
        if (code != REVLibError.kOk)
            throw new MotorRequestFailedException(this, "setD() failed during setPID() call", code) ;

        code = pid_.setFF(f) ;
        if (code != REVLibError.kOk)
            throw new MotorRequestFailedException(this, "setFF() failed during setPID() call", code) ;

        code = pid_.setIZone(0.0) ;
        if (code != REVLibError.kOk)
            throw new MotorRequestFailedException(this, "setIZone() failed during setPID() call", code) ;

        code = pid_.setOutputRange(-outmax, outmax) ;
        if (code != REVLibError.kOk)
            throw new MotorRequestFailedException(this, "setOutputRange() failed during setPID() call", code) ;

        ptype_ = type ;
    }

    /// \brief Stop the PID loop in the motor controller      
    public void stopPID() throws BadMotorRequestException {
        set(0.0) ;   
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor     
    public void set(double percent) {
        if (sim_ != null) {
            sim_power_.set(percent) ;
        } else {
            controller_.set(percent) ;
        }
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not      
    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(inverted) ;
        } else {
            controller_.setInverted(inverted);
        }

        inverted_ = inverted ;
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted        
    public boolean isInverted() {
        return inverted_ ;
    }    

    /// \brief Reapplies the inverted status of the motor.  When setInverted() is called, the inverted state of the motor
    /// is stored and this method reapplies that stored state to the motor controller.  This was put into place because some
    /// motors setup to follow other motors lost their inverted state when the robot was disabled and re-enabled.    
    // public void reapplyInverted() {
    //     if (sim_ != null) {
    //         sim_motor_inverted_.set(inverted_) ;
    //     } else {
    //         controller_.setInverted(inverted_);
    //     }
    // }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor        
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException {
        if (sim_ != null) {
            switch(mode)
            {
                case Coast:
                    sim_neutral_mode_.set(false) ;
                    break ;

                case Brake:
                    sim_neutral_mode_.set(true) ;
                    break ;
            }
        }
        else {
            switch(mode)
            {
                case Coast:
                    controller_.setIdleMode(IdleMode.kCoast) ;
                    break ;

                case Brake:
                    controller_.setIdleMode(IdleMode.kBrake) ;
                break ;
            }
        }
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader is inverted
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.      
    public void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException {
        if (sim_ == null) {
            try {
                SparkMaxMotorController other = (SparkMaxMotorController)ctrl ;
                controller_.follow(other.controller_, invert) ;
            }
            catch(ClassCastException ex)
            {
                throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
            }
        }
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type     
    public String getType() {
        String ret = null ;

        if (brushless_)
        {
            ret = "SparkMax:brushless" ;
        }
        else
        {
            ret = "SparkMax:brushed" ;
        }

        return ret ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position      
    public boolean hasPosition() {
        return brushless_ ;
    }
    
    /// \brief Returns the velocity of the motor in ticks per 100 ms.
    /// \returns the velocity of the motor in ticks per 100 ms    
    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {

        double ret = 0.0 ;

        if (sim_ != null) {
            throw new BadMotorRequestException(this, "cannot use velocity from the motor controller when simulating") ;
        }
        else {
            //
            // This comes from the motor in RPMs.  We multiply by this conversion factor to return 
            // velocity in ticks per 100 ms just like the TalonFX motors.
            //
            ret = encoder_.getVelocity() * RPM2TicksPer100MS ;
        }

        return ret ;
    }

    /// \brief Returns the position of the motor in encoder ticks
    /// \returns the position of the motor in encoder ticks
    public double getPosition() throws BadMotorRequestException {
        double ret = 0 ;

        if (!brushless_)
            throw new BadMotorRequestException(this, "brushed motor does not support getPosition()") ;

        if (sim_ != null) {
            ret = sim_encoder_.get() ;
        } else {
            ret = encoder_.getPosition() * TicksPerRevolutionValue ;
        }

        return ret ;
    }

    /// \brief Returns the number of ticks per revolution for the motor if it has an embedded encoder
    /// \returns the number of ticks per revolution for the motor if it has an embedded encoder
    public double TicksPerRevolution() throws BadMotorRequestException {
        return TicksPerRevolutionValue ;
    }

    /// \brief Reset the encoder values to zero
    public void resetEncoder() throws BadMotorRequestException {
        if (!brushless_)
            throw new BadMotorRequestException(this, "brushed motor does not support resetEncoder()") ;

        if (sim_ != null) {
            sim_encoder_.set(0.0) ;
        }
        else {
            encoder_.setPosition(0.0) ;
        }
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given        
    public void setCurrentLimit(double limit, double free) throws BadMotorRequestException {
        if (sim_ == null) {
            controller_.setSmartCurrentLimit((int)limit, (int)free) ;
        }
    }      

    /// \brief Set the open loop ramp rate for the motor
    /// \param limit the amount of time for the motor to ramp from no power to full power       
    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            controller_.setOpenLoopRampRate(limit) ;
        }
    } 

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller        
    public String getFirmwareVersion() throws BadMotorRequestException {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 24) & 0xff) + "." + String.valueOf((v >> 16) & 0xff) ;
    }

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values     
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency pos, EncoderUpdateFrequency vel) throws BadMotorRequestException {
        int p1 = 100 ;
        int p2 = 100 ;

        if (pos == EncoderUpdateFrequency.Default) {
            p2 = 20 ;
        }
        else if (pos == EncoderUpdateFrequency.Frequent) {
            p2 = 10 ;
        }

        if (vel == EncoderUpdateFrequency.Default) {
            p1 = 20 ;
        }
        else if (vel == EncoderUpdateFrequency.Frequent) {
            p1 = 10 ;
        }
        
        controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, p1) ;
        controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, p2) ;    
    }    

    public TalonFX getTalonFX() throws BadMotorRequestException {
        return null ;
    }
} ;
