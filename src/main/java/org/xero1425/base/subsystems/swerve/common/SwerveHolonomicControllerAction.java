package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public abstract class SwerveHolonomicControllerAction extends SwerveDriveAction {
    private SwerveBaseSubsystem swerve_ ;
    private HolonomicDriveController ctrl_ ;

    public SwerveHolonomicControllerAction(SwerveBaseSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        swerve_ = sub ;
        ctrl_ = createDriveController() ;
    }

    protected HolonomicDriveController controller() {
        return ctrl_ ;
    }

    protected HolonomicDriveController createDriveController() throws BadParameterTypeException, MissingParameterException {
        HolonomicDriveController ctrl = null ;
        double kp, ki, kd ;

        double maxv = swerve_.getSettingsValue("physical:max-angular-speed").getDouble() ;
        double maxa = swerve_.getSettingsValue("physical:max-angular-accel").getDouble() ;

        kp = swerve_.getSettingsValue("pid:xctrl:kp").getDouble() ;
        ki = swerve_.getSettingsValue("pid:xctrl:ki").getDouble() ;
        kd = swerve_.getSettingsValue("pid:xctrl:kd").getDouble() ;
        PIDController xctrl = new PIDController(kp, ki, kd) ;

        kp = swerve_.getSettingsValue("pid:yctrl:kp").getDouble() ;
        ki = swerve_.getSettingsValue("pid:yctrl:ki").getDouble() ;
        kd = swerve_.getSettingsValue("pid:yctrl:kd").getDouble() ;
        PIDController yctrl = new PIDController(kp, ki, kd) ;

        kp = swerve_.getSettingsValue("pid:rotation:kp").getDouble() ;
        ki = swerve_.getSettingsValue("pid:rotation:ki").getDouble() ;
        kd = swerve_.getSettingsValue("pid:rotation:kd").getDouble() ;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxv, maxa) ;
        ProfiledPIDController thetactrl = new ProfiledPIDController(kp, ki, kd, constraints) ;
        thetactrl.enableContinuousInput(-Math.PI, Math.PI);
        
        ctrl = new HolonomicDriveController(xctrl, yctrl, thetactrl) ;
        ctrl.setEnabled(true);
        
        ISettingsSupplier settings = getSubsystem().getRobot().getSettingsSupplier() ;
        String s = getSubsystem().getName() ;
        double xytol = settings.get("subsystems:" + s + ":holonomic-path-following:xy-tolerance").getDouble() ;
        double angletol = settings.get("subsystems:" + s + ":holonomic-path-following:angle-tolerance").getDouble() ;
        ctrl_.setTolerance(new Pose2d(xytol, xytol, Rotation2d.fromDegrees(angletol))) ;

        return ctrl ;
    }
}
