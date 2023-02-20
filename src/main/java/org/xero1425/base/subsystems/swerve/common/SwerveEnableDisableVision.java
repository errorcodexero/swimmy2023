package org.xero1425.base.subsystems.swerve.common;

public class SwerveEnableDisableVision extends SwerveDriveAction {
    private SwerveBaseSubsystem sub_ ;
    private boolean enable_ ;

    public SwerveEnableDisableVision(SwerveBaseSubsystem sub, boolean enable) {        
        super(sub) ;

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.enableVision(enable_) ;
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveEnableDisableVision(" + Boolean.toString(enable_) + ")" ;
    }
}
