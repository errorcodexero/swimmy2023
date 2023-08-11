package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.swerve.xeroswerve.XeroSwerveDriveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/// \file

/// \brief set the angle and the speed of each swerve module
/// This action drives each of the four modules to a specific angle, and drives the speeds of the drive
/// motors to a specific, PID controlled speed.
///

public class SwervePowerAngleAction extends SwerveDriveAction {
    static final double DefaultPlotInterval = 4.0 ;

    private double [] angles_ ;
    private double [] speeds_ ;

    private XeroTimer plot_timer_ ;
    private XeroTimer action_timer_ ;

    public SwervePowerAngleAction(SwerveBaseSubsystem subsys, double [] angles, double [] speeds) throws Exception {
        this(subsys, angles, speeds, Double.NaN) ;
    }

    public SwervePowerAngleAction(SwerveBaseSubsystem subsys, double [] angles, double [] speeds, double duration) throws Exception {
        super(subsys) ;

        if (angles.length != 4) {
            throw new Exception("SwerveAngleVelocityAction action with angles.length not equal four.") ;
        }

        if (speeds.length != 4) {
            throw new Exception("SwerveAngleVelocityAction action with angles.length not equal four.") ;
        }

        angles_ = angles.clone() ;
        speeds_ = speeds.clone() ;

        if (Double.isFinite(duration))
            action_timer_ = new XeroTimer(subsys.getRobot(), "SwervePowerAngleAction-action", duration) ;

        plot_timer_ = new XeroTimer(subsys.getRobot(), "SwerveAngleVelocityAction-timer", DefaultPlotInterval) ;
    }

    public SwervePowerAngleAction(SwerveBaseSubsystem subsys, double angle, double speed) throws Exception {
        this(subsys, angle, speed, Double.NaN) ;
    }

    public SwervePowerAngleAction(SwerveBaseSubsystem subsys, double angle, double speed, double duration) throws Exception {
        super(subsys) ;

        angles_ = new double[4] ;
        angles_[XeroSwerveDriveSubsystem.FL] = angle ;
        angles_[XeroSwerveDriveSubsystem.BL] = angle ;
        angles_[XeroSwerveDriveSubsystem.FR] = angle ;
        angles_[XeroSwerveDriveSubsystem.BR] = angle ;

        speeds_ = new double[4] ;
        for(int i = 0 ; i < 4 ; i++) 
            speeds_[i] = speed ;

        if (Double.isFinite(duration))
            action_timer_ = new XeroTimer(subsys.getRobot(), "SwervePowerAngleAction-action", duration) ;

        plot_timer_ = new XeroTimer(subsys.getRobot(), "SwerveAngleVelocityAction-plot-timer", 4) ;
    }

    void updateTargets(double[] angles, double[] speeds) {
        angles_ = angles.clone() ;
        speeds_ = speeds.clone() ;

        getSubsystem().setRawTargets(true, angles_, speeds_);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        plot_timer_.start();
        if (action_timer_ != null)
            action_timer_.start() ;

        getSubsystem().startSwervePlot("swerve");
        getSubsystem().setRawTargets(true, angles_, speeds_);
    }

    @Override
    public void run() {

        if (action_timer_ == null && plot_timer_.isExpired()) {
            //
            // The action will run forever, we will stop the plot after the DefaultPlotInterval
            //
            getSubsystem().endSwervePlot();
        }

        if (action_timer_ != null && action_timer_.isExpired()) {
            //
            // If the action has a duration, we stop the action and the plot after the duration has
            // expired
            //
            getSubsystem().endSwervePlot();
            getSubsystem().drive(new ChassisSpeeds()) ;
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        getSubsystem().endSwervePlot();
        try {
            getSubsystem().drive(new ChassisSpeeds()) ;
        }
        catch(Exception ex) {                
        }
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwervePowerAngleAction: " ;
        for(int which = 0 ; which < getSubsystem().getModuleCount()  ; which++) 
        {
            if (which != 0)
                ret += " " ;
            ret += " " + Double.toString(angles_[which]) ;
            ret += " " + Double.toString(speeds_[which]) ;
        }
        return ret ;
    }
}
