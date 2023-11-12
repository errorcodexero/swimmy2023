package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem.VisionMode;

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
        if (enable_) {
            sub_.setVisionMode(VisionMode.Normal);
        }
        else {
            sub_.setVisionMode(VisionMode.Disabled);            
        }
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveEnableDisableVision(" + Boolean.toString(enable_) + ")" ;
    }
}
