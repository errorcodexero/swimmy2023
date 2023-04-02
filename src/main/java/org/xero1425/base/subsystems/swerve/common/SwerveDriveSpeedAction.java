package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.misc.XeroTimer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveSpeedAction extends SwerveDriveAction {

    private ChassisSpeeds speed_ ;
    private XeroTimer timer_ ;
    private double time_ ;

    public SwerveDriveSpeedAction(SwerveBaseSubsystem sub, ChassisSpeeds speed, double time) {
        super(sub) ;
        speed_ = speed ;
        timer_ = new XeroTimer(sub.getRobot(), "DriveSpeedAction", time);
        time_ = time;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        timer_.start() ;
        getSubsystem().drive(speed_);
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        
        if (timer_.isExpired()) {
            setDone() ;
            getSubsystem().drive(new ChassisSpeeds());
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveSpeedAction " + speed_.toString() + ", time " + time_;
    }
}
