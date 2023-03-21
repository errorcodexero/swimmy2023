package org.xero1425.base.subsystems.oi;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveChassisSpeedAction;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveXPatternAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveDriveGamepad extends Gamepad {
    public enum SwerveButton {
        A,
        B,
        X,
        Y,
        LJoy,
        RJoy,
        RBack,
        LBack,
        RTrigger,
        LTrigger
    }

    private SwerveBaseSubsystem db_;
    private boolean invert_ ;
    private double angle_maximum_;
    private double pos_maximum_;
    private double deadband_pos_x_ ;
    private double deadband_pos_y_ ;
    private double deadband_rotate_ ;
    private double power_ ;
    private SwerveDriveChassisSpeedAction action_;
    private SwerveDriveXPatternAction x_action_;
    private SwerveButton[] reset_buttons_ ;
    private SwerveButton[] drivebase_x_buttons_;
    private boolean holding_x_;

    public SwerveDriveGamepad(OISubsystem oi, int index, SwerveBaseSubsystem drive_) throws Exception {
        super(oi, "swerve_gamepad", index);

        if (DriverStation.getStickPOVCount(getIndex()) == 0) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        if (DriverStation.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        db_ = drive_;
        reset_buttons_ = null ;
        drivebase_x_buttons_ = null;
        holding_x_ = false ;
    }

    public void invert(boolean inv) {
        invert_ = inv;
    }

    public void setSwerveResetButtons(SwerveButton[] buttons) {
        reset_buttons_ = buttons ;
    }

    public void setSwerveXButtons(SwerveButton[] buttons) {
        drivebase_x_buttons_ = buttons ;
    }

    @Override
    public void init(LoopType ltype) {
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        deadband_pos_x_ = getSubsystem().getSettingsValue("swerve_gamepad:position:deadband:x").getDouble() ;
        deadband_pos_y_ = getSubsystem().getSettingsValue("swerve_gamepad:position:deadband:y").getDouble() ;
        deadband_rotate_ = getSubsystem().getSettingsValue("swerve_gamepad:angle:deadband").getDouble() ;
        pos_maximum_ = getSubsystem().getSettingsValue("swerve_gamepad:position:maximum").getDouble();
        angle_maximum_ = getSubsystem().getSettingsValue("swerve_gamepad:angle:maximum").getDouble();
        power_ = getSubsystem().getSettingsValue("swerve_gamepad:power").getDouble();

        action_ = new SwerveDriveChassisSpeedAction(db_) ;
        x_action_ = new SwerveDriveXPatternAction(db_);
    }

    private boolean isButtonSequencePressed(SwerveButton[] buttons) {
        boolean ret = true ;

        if (buttons != null && buttons.length > 0) {
            for(SwerveButton button : buttons) {
                boolean bstate = false ;

                switch(button) {
                    case A: bstate = isAPressed() ; break ;
                    case B: bstate = isBPressed() ; break ;
                    case X: bstate = isXPressed() ; break ;
                    case Y: bstate = isYPressed() ; break ;
                    case LBack: bstate = isLBackButtonPressed() ; break ;
                    case RBack: bstate = isRBackButtonPressed() ; break ;
                    case LJoy: bstate = isLJoyButtonPressed() ; break ;
                    case RJoy: bstate = isRJoyButtonPressed() ; break ;
                    case LTrigger: bstate = isLTriggerPressed() ; break ;
                    case RTrigger: bstate = isRTriggerPressed() ; break ;
                } ;

                if (!bstate) {
                    ret = false ;
                    break ;
                }
            }
        }

        return ret;
    }

    @Override
    public void computeState() {
        super.computeState();

        if (isButtonSequencePressed(reset_buttons_)) {
            RobotSubsystem robotSubsystem = getSubsystem().getRobot().getRobotSubsystem();
            SwerveBaseSubsystem db = (SwerveBaseSubsystem)robotSubsystem.getDB() ;
            if (db != null) {
                boolean invert = false ;
                if (invert_ && DriverStation.getAlliance() == Alliance.Red) {
                    invert = true ;
                }
                db.resetPose(invert);
            }
        }

        if (isButtonSequencePressed(drivebase_x_buttons_)) {
            if (db_.getAction() != x_action_) {
                db_.setAction(x_action_);
            }
            action_.update(new ChassisSpeeds());
            holding_x_ = true ;
        }
        else {
            if (db_.getAction() != action_ && getSubsystem().getRobot().isTeleop()) {
                db_.setAction(action_);
            }
            holding_x_ = false ;
        }
    }

    @Override
    public void generateActions() {
        double ly, lx, rx ;

        if (db_ == null || !isEnabled() || holding_x_)
            return ;

        // For X axis, left is -1, right is +1
        // For Y axis, forward is -1, back is +1

        try {
            ly = DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
            lx = DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTX.value) ;
            rx = -DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;
        }
        catch(Exception ex) {
            return ;
        }

        if (invert_ && DriverStation.getAlliance() == Alliance.Red) {
            ly = -ly ;
            lx = -lx ;
        }

        double lyscaled = mapJoyStick(ly, pos_maximum_, deadband_pos_y_, power_) ;
        double lxscaled = mapJoyStick(lx, pos_maximum_, deadband_pos_x_, power_) ;
        double rxscaled = mapJoyStick(rx, angle_maximum_, deadband_rotate_, power_) ;

        if (isLTriggerPressed()) {
            lxscaled *= 0.25 ;
            lyscaled *= 0.25 ;
            rxscaled *= 0.25 ;
        }

        //
        // The rotational velocity is given by rxscaled
        // The position velocity is given by the vector (lyscaled, lxscaled)
        //
        // Note, the x and y are swapped because of the orientation of the gamepad versus the orientation of the
        // field.  The drive team is at the end of the field looking down the X axis.  So, when the Y axis on the
        // gamepad is pushed forward (negative value from the gamepad), the driver expects the robot to move along
        // the positive X axis of the field.
        //
        rxscaled *= 2.0 / Math.hypot(db_.getLength(), db_.getWidth()) / 39.37;  // 39.27 to convert meters -> inches. Original equation from SDS assumes inches.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-lyscaled, -lxscaled, rxscaled, db_.getHeading()) ;
        action_.update(speeds) ;

        // MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        // logger.startMessage(MessageType.Info);
        // logger.add("lxscaled", lxscaled) ;
        // logger.add("lyscaled", lyscaled) ;
        // logger.add("rxscaled", rxscaled) ;
        // logger.add("dbheading", db_.getHeading());
        // logger.add("gamepad: ");
        // logger.add(speeds.toString());
        // logger.endMessage();

        if (db_.getAction() != action_)
            db_.setAction(action_) ;
    }

    private double mapJoyStick(double v, double maxv, double db, double power) {
        if (Math.abs(v) < db)
            return 0.0 ;

        return Math.signum(v) * Math.pow(Math.abs(v), power) * maxv ;
    }
}
