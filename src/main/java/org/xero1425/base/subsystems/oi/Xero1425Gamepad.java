package org.xero1425.base.subsystems.oi;

import edu.wpi.first.wpilibj.DriverStation;

import org.xero1425.base.LoopType;
import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.tankdrive.TankDrivePowerAction;
import org.xero1425.base.subsystems.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This class controls interprets the input from the game pad to control the drivebase.
/// This class expects values stored in the JSON settings file.
///
///     {
///         "subsystems" : {
///             "oisubsystemname" : {
///                 "gamepadname" : {
///                     "index" : 0,
///                     "power" : {
///                         "max" : 1.0,
///                         "default" : 0.6,
///                         "nudge_straight" : 0.2,
///                         "nudge_rotate" : 0.2,
///                         "slowby" : 0.5,
///                         "tolerance" : 0.05
///                     },
///                     "turn" : {
///                         "max" : 0.8,
///                         "default" : 0.4
///                     },
///                     "zerolevel" : 0.1,
///                     "nudge_time" : 0.1
///                 }
///             }
///         }
///     }
///
public class Xero1425Gamepad extends Gamepad {
    
    // The drivebase subsystem to control
    private TankDriveSubsystem db_ ;

    // The action for nudging forward
    private Action nudge_forward_ ;

    // The action for nudging backward
    private Action nudge_backward_ ;

    // The action for nudging clockwise
    private Action nudge_clockwise_ ;

    // The action for nudging counter clockwise
    private Action nudge_counter_clockwise_ ;

    // The POV index
    private int pov_ ;

    // The default power to apply based on the joystick axis
    private double default_power_ ;

    // The max power to apply based on the joystick axis
    private double max_power_ ;

    // The default turn power to apply
    private double turn_power_ ;

    // The max turn power to apply
    private double turn_max_power_ ;

    // The factor to apply to the power when the slow button is pressed
    private double slow_factor_ ;

    // The level of the gamepad axis, below which we consider zero
    private double zero_level_ ;

    // The amount the power output must change before we assign new power valus
    private double tolerance_ ;

    // The current left drivebase power
    private double left_ ;

    // The current right drivebase power
    private double right_ ;

    /// \brief Create a new TankDrive gamepad device
    /// \param oi the subsystems that owns this device
    /// \param index the index to use when access this device in the WPILib library
    /// \param drive the drivebase to control
    public Xero1425Gamepad(OISubsystem oi, int index, DriveBaseSubsystem drive) throws Exception {
        super(oi, "xero1425_gamepad", index);

        if (DriverStation.getStickPOVCount(getIndex()) == 0) {
            throw new Exception("invalid gamepad for Xero1425Gamepad");
        }

        if (DriverStation.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            throw new Exception("invalid gamepad for Xero1425Gamepad");
        }

        db_ = (TankDriveSubsystem)drive;

        if (db_ == null) {
            throw new Exception("invalid drivebase for Xero1425Gamepad - expected tankdrive");
        }
    }

    /// \brief initialize the gamepad per robot mode, does nothing
    @Override
    public void init(LoopType ltype) {
    }

    /// \brief create the required static actions
    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {

        default_power_ = getSubsystem().getSettingsValue(getName() + ":power:default").getDouble();
        max_power_ = getSubsystem().getSettingsValue(getName() + ":power:max").getDouble();
        turn_power_ = getSubsystem().getSettingsValue(getName() + ":turn:default").getDouble();
        turn_max_power_ = getSubsystem().getSettingsValue(getName() + ":turn:max").getDouble();
        slow_factor_ = getSubsystem().getSettingsValue(getName() + ":power:slowby").getDouble();
        zero_level_ = getSubsystem().getSettingsValue(getName() + ":zerolevel").getDouble();

        tolerance_ = getSubsystem().getSettingsValue(getName() + ":power:tolerance").getDouble();

        double nudge_straight = getSubsystem().getSettingsValue(getName() + ":power:nudge_straight").getDouble();
        double nudge_rotate = getSubsystem().getSettingsValue(getName() + ":power:nudge_rotate").getDouble();
        double nudge_time = getSubsystem().getSettingsValue(getName() + ":nudge_time").getDouble();

        nudge_forward_ = new TankDrivePowerAction(db_, nudge_straight, nudge_straight, nudge_time);
        nudge_backward_ = new TankDrivePowerAction(db_, -nudge_straight, -nudge_straight, nudge_time);
        nudge_clockwise_ = new TankDrivePowerAction(db_, nudge_rotate, -nudge_rotate, nudge_time);
        nudge_counter_clockwise_ = new TankDrivePowerAction(db_, -nudge_rotate, nudge_rotate, nudge_time);
    }

    /// \brief generate the actions for the drivebase for the current robot loop
    @Override
    public void generateActions() {
        SequenceAction seq = new SequenceAction(getSubsystem().getRobot().getMessageLogger()) ;
        if (db_ == null || !isEnabled())
            return ;

        try {
            POVAngle povvalue ;

            if (pov_ == -1)
                povvalue = POVAngle.NONE ;
            else
                povvalue = POVAngle.fromInt(DriverStation.getStickPOV(getIndex(), pov_)) ;

            double ly = DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
            double rx = DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

            if (povvalue == POVAngle.LEFT)
                seq.addSubActionPair(db_, nudge_counter_clockwise_, false);
            else if (povvalue == POVAngle.RIGHT)
                seq.addSubActionPair(db_, nudge_clockwise_, false);       
            else if (povvalue == POVAngle.UP)
                seq.addSubActionPair(db_, nudge_forward_, false);  
            else if (povvalue == POVAngle.DOWN)
                seq.addSubActionPair(db_, nudge_backward_, false);                                       
            else {
                double left, right ;

                if (Math.abs(ly) < zero_level_ && Math.abs(rx) < zero_level_) {
                    left = 0.0 ;
                    right = 0.0; 
                }
                else {
                    double boost = DriverStation.getStickAxis(getIndex(), AxisNumber.LTRIGGER.value) ;
                    boolean slow = isLBackButtonPressed() ;

                    double power = scalePower(-ly, boost, slow) ;
                    double spin = (Math.abs(rx) > 0.01) ? scaleTurn(rx, boost, slow) : 0.0 ;

                    left = power + spin ;
                    right = power - spin ;                    
                }

                if (Math.abs(left - left_) > tolerance_ || Math.abs(right - right_) > tolerance_)
                {
                    TankDrivePowerAction act = new TankDrivePowerAction(db_, left, right) ;
                    seq.addSubActionPair(db_, act, false);
                    left_ = left ;
                    right_ = right ;
                }
            }
        }
        catch(Exception ex) {
            //
            // This should never happen
            //
        }

        if (seq.size() == 1)
            db_.setAction(seq.get(0)) ;
    }

    private double scalePower(double axis, double boost, boolean slow) {
        double base = default_power_ + (max_power_ - default_power_) * boost ;
        double slowdown = slow ? default_power_ * slow_factor_ : 0.0 ;
        return axis * (base - slowdown) ;
    }

    private double scaleTurn(double axis, double boost, boolean slow) {
        double base = turn_power_ + (turn_max_power_ - turn_power_) * boost ;
        double slowdown = slow ? turn_power_ * slow_factor_ : 0.0 ;
        return axis * (base - slowdown) ;
    }
}