package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.swerve.xeroswerve.XeroSwerveDriveSubsystem;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwervePathFollowAction extends SwerveDriveAction {
    private int index_;
    private String pathname_;
    private XeroPath path_;
    private double[] angles_;
    private double[] speeds_;

    public SwervePathFollowAction(XeroSwerveDriveSubsystem drive, String path) {

        super(drive);

        pathname_ = path;
        path_ = null;

        angles_ = new double[4] ;
        speeds_ = new double[4] ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        index_ = 0;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);

        XeroPathSegment fl = path_.getSegment(XeroSwerveDriveSubsystem.FL, 0) ;
        XeroPathSegment fr = path_.getSegment(XeroSwerveDriveSubsystem.FR, 0) ;
        XeroPathSegment bl = path_.getSegment(XeroSwerveDriveSubsystem.BL, 0) ;
        XeroPathSegment br = path_.getSegment(XeroSwerveDriveSubsystem.BR, 0) ;

        double x = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
        double y = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
        double heading = fl.getHeading() ;

        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(heading)) ;
        getSubsystem().setPose(pose);

        getSubsystem().startSwervePlot("SwervePathFollowAction") ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        if (index_ < path_.getTrajectoryEntryCount())
        {
            XeroPathSegment fl = path_.getSegment(XeroSwerveDriveSubsystem.FL, index_) ;
            XeroPathSegment fr = path_.getSegment(XeroSwerveDriveSubsystem.FR, index_) ;
            XeroPathSegment bl = path_.getSegment(XeroSwerveDriveSubsystem.BL, index_) ;
            XeroPathSegment br = path_.getSegment(XeroSwerveDriveSubsystem.BR, index_) ;

            angles_[XeroSwerveDriveSubsystem.FL] = fl.getHeading() ;
            angles_[XeroSwerveDriveSubsystem.FR] = fr.getHeading() ;
            angles_[XeroSwerveDriveSubsystem.BL] = bl.getHeading() ;
            angles_[XeroSwerveDriveSubsystem.BR] = br.getHeading() ;

            speeds_[XeroSwerveDriveSubsystem.FL] = fl.getVelocity() ;
            speeds_[XeroSwerveDriveSubsystem.FR] = fr.getVelocity() ;
            speeds_[XeroSwerveDriveSubsystem.BL] = bl.getVelocity() ;
            speeds_[XeroSwerveDriveSubsystem.BR] = br.getVelocity() ;

            getSubsystem().setRawTargets(false, angles_, speeds_);
            index_++ ;
        }

        if (index_ == path_.getTrajectoryEntryCount())
        {
            getSubsystem().drive(new ChassisSpeeds()) ;
            getSubsystem().endSwervePlot();
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        setDone();
    }

    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveDriveFollowPathAction-" + pathname_ ;
        return ret ;
    }
}
