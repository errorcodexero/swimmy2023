package org.xero1425.base.subsystems.oi;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

/// \file

/// \brief This class controls interprets the input from the game pad to control the drivebase.
/// This class expects values stored in the JSON settings file.
///
///     {
///         "subsystems" : {
///             "oisubsystemname" : {
///                 "gamepadname" : {
///                 }
///             }
///         }
///     }
///
public class StandardGamepad extends Gamepad {
    
    // The drivebase subsystem to control
    private TankDriveSubsystem db_ ;

    // The current left drivebase power
    private double left_ ;

    // The current right drivebase power
    private double right_ ;

    // The deadband in the joysticks
    private double deadband_ ;

    // The delta between the desired power and actual power before we update the drivebase
    private double epslion_ ;

    // The maximum increase in power to the motors in any one robot loop
    private double max_increase_ ;

    // If true, use the max increase value above
    private boolean use_max_increase_ ;

    private double max_incr_decr_threshold_ ;

    // The maximum increase in power to the motors in any one robot loop
    private double max_decrease_ ;

    // If true, use the max increase value above
    private boolean use_max_decrease_ ;

    private double power_ ;

    /// \brief Create a new TankDrive gamepad device
    /// \param oi the subsystems that owns this device
    /// \param index the index to use when access this device in the WPILib library
    /// \param drive the drivebase to control
    public StandardGamepad(OISubsystem oi, int index, DriveBaseSubsystem drive) throws Exception {
        super(oi, "standard_gamepad", index);

        db_ = (TankDriveSubsystem)drive;

        if (db_ == null) {
            throw new Exception("invalid drivebase for StandardGamepad - expected tankdrive");
        }
        
        use_max_increase_ = false ;
    }

    /// \brief initialize the gamepad per robot mode, does nothing
    @Override
    public void init(LoopType ltype) {
    }

    public TankDriveSubsystem getDB() {
        return db_ ;
    }

    /// \brief create the required static actions
    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        deadband_ = getSubsystem().getSettingsValue(getName() + ":deadband").getDouble();
        epslion_ = getSubsystem().getSettingsValue(getName() + ":epsilon").getDouble() ;
        max_increase_ = getSubsystem().getSettingsValue(getName() + ":max-increase").getDouble() ;
        max_decrease_ = getSubsystem().getSettingsValue(getName() + ":max-decrease").getDouble() ;
        max_incr_decr_threshold_ = getSubsystem().getSettingsValue(getName() + ":incr-decr-threshold").getDouble() ;
        power_ = getSubsystem().getSettingsValue(getName() + ":exponent").getDouble() ;

        if (max_increase_ > 0.01) {
          use_max_increase_ = true ;
          max_increase_ = 1.0 / (max_increase_ / db_.getRobot().getPeriod()) ;
        }

        if (max_decrease_ > 0.01) {
          use_max_decrease_ = true ;
          max_decrease_ = 1.0 / (max_decrease_ / db_.getRobot().getPeriod()) ;
        }
    }

    /// \brief generate the actions for the drivebase for the current robot loop
    @Override
    public void generateActions() {
        double xSpeed, zRotation ;

        if (db_ == null || isEnabled() == false) {
          return ;
        }

        try {
            xSpeed = -DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
            zRotation = DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;
        }
        catch(Exception ex) {
            return ;
        }
        
        xSpeed = MathUtil.applyDeadband(xSpeed, deadband_) ;
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0) ;
        xSpeed = Math.copySign(Math.pow(Math.abs(xSpeed), power_), xSpeed) ;

        zRotation = MathUtil.applyDeadband(zRotation, deadband_) ;
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0) ;
        zRotation = Math.copySign(Math.pow(Math.abs(zRotation), power_), zRotation) ;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed) ;
        double leftSpeed, rightSpeed ;

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
              leftSpeed = maxInput;
              rightSpeed = xSpeed - zRotation;
            } else {
              leftSpeed = xSpeed + zRotation;
              rightSpeed = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
              leftSpeed = xSpeed + zRotation;
              rightSpeed = maxInput;
            } else {
              leftSpeed = maxInput;
              rightSpeed = xSpeed - zRotation;
            }
        }

        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }

        if (Math.abs(leftSpeed - left_) > epslion_ || Math.abs(rightSpeed - right_) > epslion_) {
          
          if (use_max_increase_ && Math.abs(zRotation) < max_incr_decr_threshold_) {
            if (leftSpeed - left_ > max_increase_) {
              leftSpeed = left_ + max_increase_ ;
            }

            if (rightSpeed - right_ > max_increase_) {
              rightSpeed = right_ + max_increase_ ;
            }
          }

          if (use_max_decrease_ && Math.abs(zRotation) < max_incr_decr_threshold_) {
            if (left_- leftSpeed > max_decrease_) {
              leftSpeed = left_ - max_decrease_ ;
            }

            if (right_ - rightSpeed > max_decrease_) {
              rightSpeed = right_ - max_decrease_ ;
            }            
          }

          db_.setPower(leftSpeed, rightSpeed) ;
          left_ = leftSpeed ;
          right_ = rightSpeed ;
        }
    }
}
