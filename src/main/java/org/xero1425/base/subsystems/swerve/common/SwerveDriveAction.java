package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.swerve.xeroswerve.XeroSwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveDriveAction extends Action {
    private SwerveBaseSubsystem swerve_drive_ ;
    private double circum_ ;

    /// \brief Create the object holding a reference to the subsystem
    /// \param drive the tankdrive subsystem
    public SwerveDriveAction(SwerveBaseSubsystem drive) {
        super(drive.getRobot().getMessageLogger()) ;
        swerve_drive_ = drive ;

        //
        // We approximate the circumference of the robot by averaging the length and width and applying
        // the CIRCUM = 2 * PI * R.  Calculating the circumference of an ellipse is more computational intensive
        // and really does not get us that much.
        //
        circum_ = (swerve_drive_.getLength() + swerve_drive_.getWidth()) * Math.PI / 2.0 ;
    }

    /// \brief return the tank drive subsystem
    /// returns the tank drive subsystem
    public SwerveBaseSubsystem getSubsystem() {
        return swerve_drive_ ;
    }

    protected Pose2d combinePose(Pose2d fl, Pose2d fr, Pose2d bl, Pose2d br) {
        double x = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
        double y = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
        double h = (fl.getRotation().getDegrees() + fr.getRotation().getDegrees()  + bl.getRotation().getDegrees()  + br.getRotation().getDegrees()) / 4.0 ;

        return new Pose2d(x, y, Rotation2d.fromDegrees(h)) ;
    }

    protected Translation2d rotateVector(Translation2d vec, double angle) {
        double rads = angle / 180.0 * Math.PI ;
        return new Translation2d(vec.getX() * Math.cos(rads) - vec.getY() * Math.sin(rads), vec.getY() * Math.cos(rads) + vec.getX() * Math.sin(rads)) ;
    }

    protected double createRotAngle(int which) {
        double angle = 0.0 ;
        double phi = getSubsystem().getPHI() ;

        switch(which) {
            case XeroSwerveDriveSubsystem.FL:
                angle = 180 - phi ;
                break ;
            case XeroSwerveDriveSubsystem.FR:
                angle = phi ;
                break ;
            case XeroSwerveDriveSubsystem.BL:
                angle = -180 + phi ;
                break ;
            case XeroSwerveDriveSubsystem.BR:
                angle = -phi ;
                break ;                                                
        }

        return angle ;
    }

    protected Translation2d createRotVector(int which, double rot) {
        double linear = rot / 360.0 * circum_ / 2 ;
        double angle = createRotAngle(which);
        angle = angle / 180.0 * Math.PI ;
        return new Translation2d(Math.cos(angle) * linear, Math.sin(angle) * linear) ;
    }

    protected Translation2d addVectors(Translation2d v1, Translation2d v2) {
        return new Translation2d(v1.getX() + v2.getX(), v1.getY() + v2.getY()) ;
    }
}
