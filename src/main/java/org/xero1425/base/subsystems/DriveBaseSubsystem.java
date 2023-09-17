package org.xero1425.base.subsystems;

import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.RomiGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.gyro.Pigeon2Gyro;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class DriveBaseSubsystem extends Subsystem {
    private XeroGyro gyro_;

    public DriveBaseSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name);

        MessageLogger logger = getRobot().getMessageLogger();

        String gyrotype = getSettingsValue("hw:gyro:type").getString();
        double startup = getSettingsValue("hw:gyro:start-time").getDouble() ;

        if (gyrotype.equals("navx")) {
            gyro_ = new NavxGyro();
        } else if (gyrotype.equals("LSM6DS33")) {
            gyro_ = new RomiGyro();
        }
        else if (gyrotype.equals("pigeon2")) {
            int canid = getSettingsValue("hw:gyro:canid").getInteger() ;
            gyro_ = new Pigeon2Gyro(canid) ;
        }
        else {
            String msg = "the gyro type '" + gyrotype + "' is not valid.  Only 'navx' and 'LSM6D33' are supported" ;
            throw new Exception(msg) ;
        }

        double start = getRobot().getTime();
        while (getRobot().getTime() - start < startup) {
            if (gyro().isConnected())
                break;
        }

        if (!gyro_.isConnected()) {
            logger.startMessage(MessageType.Error);
            logger.add("Gyro is not connected");
            logger.endMessage();

            String msg = "cannot connect to the gyro of type '" + gyrotype + "' in the start time" ;
            throw new Exception(msg) ;
        }
    }

    public void computeMyState() throws Exception  {
        putDashboard("heading", DisplayType.Always, gyro_.getYaw());
    }

    public double getYaw() {
        return gyro_.getYaw() ;
    }

    public double getPitch() {
        return gyro_.getPitch() ;
    }
    
    public double getRoll() {
        return gyro_.getRoll() ;
    }    

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro_.getYaw()) ;
    }

    public static Pose2d segmentToPose(XeroPathSegment seg) {
        return new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
    }

    /// \brief returns true to indicate this is a drivebase
    /// \returns true to indicate this is a drivebase
    public boolean isDB() {
        return true;
    }

    public abstract Pose2d getPose() ;
    public abstract void setPose(Pose2d p) ;

    public abstract double getVelocity() ;
    public abstract double getRotationalVelocity() ;

    public XeroGyro gyro() {
        return gyro_ ;
    }
}
