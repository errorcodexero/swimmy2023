package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.LoopType;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorGroupController;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

/// \file

/// \brief This class is a subsystem that contains a single motor.  The motor is defined in the
/// settings file using the MotorFactory.  This means the motor can be a single motor, or a group
/// of mechanically connected motors working together.  See the class MotorFactory for more information
/// on the format of the JSON data to define a motor.
public class MotorSubsystem extends Subsystem
{
    // Used to compare doubles.  If the difference is less than this, they are considered equal
    private static final double epsilon = 1e-3 ;

    // The motor controller
    private MotorController controller_ ;

    // The current power applied to the motor
    private double power_ ;

    /// \brief Create a new motor subsystem.
    /// The settings file entry will be "subsystems:NAME:hw:motors", where NAME is the name
    /// of the subsystem.
    /// \param parent the parent that manages this subsystem
    /// \param name the name of this subsystem (used to access settings from the settings file)
    public MotorSubsystem(Subsystem parent, String name) {
        super(parent, name) ;

        String mname = "subsystems:" + name + ":hw:motors" ;
        controller_ = getRobot().getMotorFactory().createMotor(name, mname) ;
        if (controller_ == null)
        {
            getRobot().getMessageLogger().startMessage(MessageType.Fatal) ;
            getRobot().getMessageLogger().add("could not create motor for name '" + mname + "'") ;
            getRobot().getMessageLogger().endMessage();
        }
    }

    /// \brief Returns true if the motor is running.  Running is defined as a power greater
    /// than 1e-3.
    /// \returns true if the motor is running
    public boolean isRunning() {
        return Math.abs(power_) > epsilon ;
    }

    /// \brief Called when the robot changes into a new mode (auto, teleop, disabled, etc.).
    /// This method is used to reapply the inverted status of the motor as some of the motor controllers
    /// lose the inverstad status when they are disabled.
    /// \param ltype the new mode for the robot.
    @Override
    public void init(LoopType ltype) {
        super.init(ltype) ;
        // try {
        //     controller_.reapplyInverted();
        // }
        // catch(BadMotorRequestException | MotorRequestFailedException ex) {
        //     MessageLogger logger = getRobot().getMessageLogger() ;
        //     logger.startMessage(MessageType.Error) ;
        //     logger.add("subsystem ").addQuoted(getName()).add(": cannot reapply inverted state -").add(ex.getMessage()).endMessage();
        // }
    }

    /// \brief Set the motor power to 0 to ensure the motor does not start when the robot is enabled.
    @Override
    public void postHWInit() throws Exception {
        super.postHWInit();

        setPower(0.0) ;
    }

    /// \brief Reset the subsystem by setting the power to zero
    @Override
    public void reset() {
        super.reset() ;
        setPower(0.0) ;
    }

    /// \brief Return the value of a property for this subsystem.  The only
    /// supported property for the MotorSubsystem is "power".
    /// \param name the name of the property.
    public SettingsValue getProperty(String name) {
        if (name.equals("power"))
            return new SettingsValue(getPower()) ;

        return null ;
    }

    /// \brief Returns the current motor power
    /// \returns the current motor power.
    public double getPower() {
        return power_ ;
    }

    /// \brief Returns the motor controller object for this subsystem.  If the controller is a group
    /// controller, the first motor (the leader) in the group is returned.  It is guarenteed that a
    /// real motor controller is returned.
    /// \returns the motor controller object for this subsystem.
    public MotorController getMotorController() {
        MotorController ret = controller_ ;

        if (controller_ instanceof MotorGroupController) {
            MotorGroupController group = (MotorGroupController)controller_ ;
            ret = group.getMotor(0) ;
        }

        return ret ;
    }

    /// \brief set the power for the motor
    /// \param p the power for the motor
    public void setPower(double p) {
        try {
            // The limitPower method can be overridden in a derived class to place limits on the
            // power ever supplied to the motor
            power_ = limitPower(p) ;
            controller_.set(power_) ;
        }
        catch(BadMotorRequestException|MotorRequestFailedException ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("subsystem ").addQuoted(getName()).add(": cannot set power -").add(ex.getMessage()).endMessage();
        }
    }

    // \brief limits the power that is ever applied to the motor.  
    /// This method returns the requested power so does not really limit anything.  However, derived classes can implement
    /// this method to provide limits to the power applied.
    protected double limitPower(double p) {
        return p ;
    }
} ;