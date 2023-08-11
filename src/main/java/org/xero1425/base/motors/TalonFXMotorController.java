package org.xero1425.base.motors;

import com.ctre.phoenix.ErrorCode;

/// \file
/// This file contains the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonFX,
/// the TalonSRX, and the VictorSPX.
///

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/// \brief This class is MotorController class that supports the TalonFX motors.   This class is a
/// wrapper for the TalonFX class that provides the interface that meets the requirements of the
/// MotorController base class.
public class TalonFXMotorController extends MotorController
{
    private final static double TicksPerRevolutionValue = 2048.0 ;

    private TalonFX controller_ ;                       // The base motor controller object
    private boolean inverted_ ;                         // If true, the motor is inverted
    private PidType type_ ;                             // For a PID in the controller, the type of PID (position vs velocity)

    /// \brief the name of the device when simulating
    public final static String SimDeviceName = "CTREMotorController" ;

    /// \brief the timeout for requests to the
    private final int ControllerTimeout = 250 ;

    /// \brief Create a new TalonFX Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller
    public TalonFXMotorController(String name, String bus, int canid, boolean leader) throws MotorRequestFailedException {
        super(name, canid) ;

        inverted_ = false ;
        type_ = PidType.None ;

        controller_ = new TalonFX(canid, bus) ;
        controller_.configFactoryDefault() ;
        
        controller_.configVoltageCompSaturation(11.0, ControllerTimeout) ;
        controller_.enableVoltageCompensation(true);

        //
        // Status frame 1 is default 10ms.  It reports motor output voltage, fault information,
        // and limit switch information.  This can be slowed way down
        //
        if (leader) {
            controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, ControllerTimeout) ;
        }
        else {
            controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, ControllerTimeout) ;
        }

        controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, ControllerTimeout) ;
    }

    public void setNeutralDeadband(double value) {
        if (controller_ != null) {
            controller_.configNeutralDeadband(value) ;
        }
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
    public boolean hasPID(PidType type) throws BadMotorRequestException {
        return true ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller
    public void setTarget(double target) throws BadMotorRequestException {
        if (type_ == PidType.None)
            throw new BadMotorRequestException(this, "calling setTarget() before calling setPID()");

        if (type_ == PidType.Velocity)
            controller_.set(TalonFXControlMode.Velocity, target) ;
        else if (type_ == PidType.Position)
            controller_.set(TalonFXControlMode.Position, target) ;
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// NOTE: The values are relative to ticks per 100ms and then multiplied by 1023
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {

        ErrorCode code ;

        code = controller_.config_kP(0, p, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kP() call failed during setPID() call. Code: " + code.toString(), code) ;

        code = controller_.config_kI(0, i, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kI() call failed during setPID() call. Code: " + code.toString(), code) ;

        code = controller_.config_kD(0, d, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kD() call failed during setPID() call. Code: " + code.toString(), code) ;

        code = controller_.config_kF(0, f, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kF() call failed during setPID() call. Code: " + code.toString(), code) ;

        code = controller_.configClosedLoopPeakOutput(0, outmax, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE configClosedLoopPeakOutput() call failed during setPID() call. Code: " + code.toString(), code) ;

        type_ = type ;
    }

    /// \brief Stop the PID loop in the motor controller
    public void stopPID() throws BadMotorRequestException {
        controller_.set(ControlMode.PercentOutput, 0.0) ;
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor
    public void set(double percent) {
        controller_.set(ControlMode.PercentOutput, percent) ;
    }

    /// \brief Set the motor to invert the direction of motion
    /// \param inverted if true invert the direction of motion, otherwise do not
    public void setInverted(boolean inverted) {
        controller_.setInverted(inverted);
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted
    public boolean isInverted() {
        return inverted_ ;
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException {
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

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param leader if true, the leader if inverted
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.
    public void follow(MotorController ctrl, boolean leader, boolean invert) throws BadMotorRequestException {
        try {
            TalonFXMotorController other = (TalonFXMotorController)ctrl ;
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

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type
    public String getType() {
        return "TalonFX" ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position
    public boolean hasPosition() {
        return true ;
    }
    
    /// \brief Return the velocity of the motor from the PID loop running in the controller
    /// \returns the velocity of the motor from the PID loop running in the controller
    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        double ret = controller_.getSelectedSensorVelocity() * 10 ;
        return ret ;
    }

    /// \brief Returns the position of the motor in motor units.
    /// \returns the position of the motor in motor units
    public double getPosition() throws BadMotorRequestException {
        double ret = 0 ;

        TalonFX fx = (TalonFX)controller_ ;
        ret = fx.getSelectedSensorPosition() ;

        return ret ;
    }

    /// \brief Returns the number of ticks per revolution for the motor if it has an embedded encoder
    /// \returns the number of ticks per revolution for the motor if it has an embedded encoder
    public double TicksPerRevolution() throws BadMotorRequestException {
        return TicksPerRevolutionValue ;
    }

    /// \brief Reset the encoder values to zero
    public void resetEncoder() throws BadMotorRequestException {
        TalonFX fx = (TalonFX)controller_ ;
        fx.setSelectedSensorPosition(0) ;
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given
    public void setCurrentLimit(double limit, double free) throws BadMotorRequestException {
        TalonFX fx = (TalonFX)controller_ ;
        SupplyCurrentLimitConfiguration cfg = new SupplyCurrentLimitConfiguration(true, limit, limit, 1) ;
        fx.configSupplyCurrentLimit(cfg) ;
    }

    /// \brief Set the open loop ramp rate for the motor
    /// \param limit the amount of time for the motor to ramp from no power to full power
    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        TalonFX fx = (TalonFX)controller_ ;
        fx.configOpenloopRamp(limit, 20) ;
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
        int interval = 255 ;

        if (pos == EncoderUpdateFrequency.Frequent || vel == EncoderUpdateFrequency.Frequent) {
            interval = 5 ;
        }
        else if (pos == EncoderUpdateFrequency.Default || vel == EncoderUpdateFrequency.Default) {
            interval = 20 ;
        }

        if (controller_ != null) {
            controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, interval) ;
        }
    }

    public TalonFX getTalonFX() throws BadMotorRequestException {
        return controller_ ;
    }
} ;
