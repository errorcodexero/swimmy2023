package org.xero1425.base.subsystems.swerve.common;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveChassisSpeedAction extends SwerveDriveAction {

    private ChassisSpeeds speed_ ;

    public SwerveDriveChassisSpeedAction(SwerveBaseSubsystem sub) {
        super(sub) ;

        speed_ = new ChassisSpeeds() ;
    }

    public void update(ChassisSpeeds speed) {
        speed_ = speed ;
        getSubsystem().drive(speed) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        getSubsystem().drive(speed_) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void cancel() {
        super.cancel();
        try {

            getSubsystem().drive(new ChassisSpeeds()) ;
        }
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveChassisSpeedAction" ;
    }
}
