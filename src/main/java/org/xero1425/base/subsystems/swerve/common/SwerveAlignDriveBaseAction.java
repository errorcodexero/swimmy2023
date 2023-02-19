package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.IVisionAlignmentData;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveAlignDriveBaseAction extends SwerveDriveAction {

    private IVisionAlignmentData vision_ ;
    private double threshold_ ;
    private PIDCtrl pid_ ;

    public SwerveAlignDriveBaseAction(SwerveBaseSubsystem sub, IVisionAlignmentData vision, double threshold) throws MissingParameterException, BadParameterTypeException {
        super(sub);

        vision_ = vision;

        threshold_ = threshold;

        String name = "subsystems:" + sub.getName() + "align-pid" ;
        pid_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), name, true);
    }

    @Override
    public void start() throws Exception {
        super.start() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (Math.abs(vision_.getTX()) < threshold_) {
            getSubsystem().drive(new ChassisSpeeds());
            setDone();
        }
        else {
            double out = pid_.getOutput(0.0, vision_.getTX(), getSubsystem().getRobot().getDeltaTime());
            ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, out);
            getSubsystem().drive(speeds);
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveAlignDriveBaseAction(" + threshold_ + ")" ;
    }   
}
