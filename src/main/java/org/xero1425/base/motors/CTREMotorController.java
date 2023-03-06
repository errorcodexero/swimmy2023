package org.xero1425.base.motors;

/// \file
/// This file conatins the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonSRX, 
/// and the VictorSPX.
///

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.ErrorCode;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \brief This class is MotorController class that supports the TalonSRX, and the VictorSPX motors.
public class CTREMotorController extends MotorController
{  
    private BaseMotorController controller_ ;           // The motor controller class from the CTRE library
    private boolean inverted_ ;                         // True if the direction of the motor is inverted
    private MotorType type_ ;                           // The actual motor type, either a TalonSRX or a VictorSPX


    private SimDevice sim_ ;                            // The device when simulating
    private SimDouble sim_power_ ;                      // The current power for the simulated motor
    private SimBoolean sim_motor_inverted_ ;            // The inverted status of the motor when simulating
    private SimBoolean sim_neutral_mode_ ;              // The neutral mode for the motor when simulating

    /// \brief The name of the device when simulating
    public final static String SimDeviceName = "CTREMotorController" ;

    /// \brief the timeout when making motor controller calls where a response is required
    private final int ControllerTimeout = 250 ;

    /// \brief The type of the physical motor controller
    public enum MotorType
    {
        TalonSRX,           ///< A Talon SRX motor controller
        VictorSPX,          ///< A Victor SPX motor controller
    } ;
    
    /// \brief Create a new Talon SRX or Victor SPX Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller
    /// \param type the type of the motor controller
    public CTREMotorController(String name, int index, MotorType type) throws MotorRequestFailedException {
        super(name) ;

        inverted_ = false ;
        type_ = type ;

        if (RobotBase.isSimulation()) {
            //
            // If we are simulating, create the simulation device and values
            //
            sim_ = SimDevice.create(SimDeviceName, index) ;

            sim_power_ = sim_.createDouble(MotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(MotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(MotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;
            sim_.createBoolean(MotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, true) ;

        }
        else {
            //
            // Now simulating, create a real motor
            //
            ErrorCode code ;

            sim_ = null ;
            sim_power_ = null ;

            switch(type_)
            {
                case TalonSRX:
                    controller_ = new TalonSRX(index) ;
                    break ;

                case VictorSPX:
                    controller_ = new VictorSPX(index) ;
                    break ;
            }

            //
            // Restore everything to the factor default
            //
            code = controller_.configFactoryDefault(ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configFactoryDefault() call failed during initialization", code) ;

            //
            // All motors have voltage compensation set at 11.0 volts
            //
            code = controller_.configVoltageCompSaturation(11.0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configVoltageCompSaturation() call failed during initialization", code) ;
            controller_.enableVoltageCompensation(true);
        }
    }

    public void setNeutralDeadband(double value) {
        controller_.configNeutralDeadband(value) ;
    }

    /// \brief should return the velocity if the motor controller can be configured with the PID loop in the controller.  This is
    /// not supports for the TalonSRX or VictorSPX so this method throws an exception
    /// \throws BadMotorRequestException always
    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "PID control not implemented") ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller    
    public double getInputVoltage() throws BadMotorRequestException {
        return controller_.getBusVoltage() ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor       
    public double getAppliedVoltage() throws BadMotorRequestException {
        return controller_.getMotorOutputVoltage() ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller    
    public boolean hasPID() throws BadMotorRequestException {
        return false ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller        
    public void setTarget(double target) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "PID control not implemented");
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller     
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {
        throw new BadMotorRequestException(this, "PID control not implemented");        
    }

    /// \brief Stop the PID loop in the motor controller     
    public void stopPID() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "PID control not implemented");   
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor  
    public void set(double percent) {
        if (sim_ != null) {
            sim_power_.set(percent) ;
        }
        else {
            controller_.set(ControlMode.PercentOutput, percent) ;
        }
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not   
    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(true) ;
        }
        else {
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
    //     }
    //     else {
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
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
                    break ;

                case Brake:
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                break ;            
            }
        }
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader if inverted versus normal
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following. 
    public void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException {
        if (sim_ == null) {
            if (invert)
                throw new BadMotorRequestException(this, "cannot follow another controller inverted") ;

            try {
                CTREMotorController other = (CTREMotorController)ctrl ;
                controller_.follow(other.controller_) ;

                if (leader != invert)
                    controller_.setInverted(InvertType.OpposeMaster) ;
                else
                    controller_.setInverted(InvertType.FollowMaster);
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

        switch(type_)
        {
            case TalonSRX:
                ret = "TalonSRX" ;
                break ;

            case VictorSPX:
                ret = "VictorSPX" ;
                break ;
        }

        return ret ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position 
    public boolean hasPosition() {
        return false ;
    }

    /// \brief Returns the position of the motor in motor units.  There is no integrated sensor, so an exception is thrown
    /// \returns the position of the motor in motor units   
    public double getPosition() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support getPosition()") ;
    }

    /// \brief Reset the encoder values to zero        
    public void resetEncoder() throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "motor does not support getPosition()") ;
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given    
    @Override
    public void setCurrentLimit(double limit, double free) throws BadMotorRequestException {
        if (sim_ == null) {
            if (controller_ instanceof TalonSRX)
            {
                TalonSRX srx = (TalonSRX)controller_ ;
                SupplyCurrentLimitConfiguration cfg = new SupplyCurrentLimitConfiguration();
                cfg.currentLimit = limit ;
                cfg.enable = true ;
                srx.configSupplyCurrentLimit(cfg) ;
            }
            else
            {
                throw new BadMotorRequestException(this, "motor does not support setCurrentLimit()") ;
            }

        }
    }     

    /// \brief Set the open loop ramp rate for the motor
    /// \param limit the amount of time for the motor to ramp from no power to full power     
    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            if (controller_ instanceof VictorSPX)
            {
                VictorSPX spx = (VictorSPX)controller_ ;
                spx.configOpenloopRamp(limit) ;
            }
            else if (controller_ instanceof TalonSRX)
            {
                TalonSRX srx = (TalonSRX)controller_ ;
                srx.configOpenloopRamp(limit) ;
            }
            else
            {
                throw new BadMotorRequestException(this, "motor does not support setCurrentLimit()") ;
            }            
        }
    }  

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller   
    public String getFirmwareVersion() throws BadMotorRequestException {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;
    }
    
    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values    
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency pos, EncoderUpdateFrequency vel) throws BadMotorRequestException {
        throw new BadMotorRequestException(this, "this motor does not contains integrated encoders") ;
    }
} ;

