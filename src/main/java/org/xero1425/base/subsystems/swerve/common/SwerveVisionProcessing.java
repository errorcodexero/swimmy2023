package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.IVisionLocalization;
import org.xero1425.base.IVisionLocalization.LocationData;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public class SwerveVisionProcessing {


    private int logger_id_ ;

    private SwerveBaseSubsystem sub_ ;
    private IVisionLocalization vision_ ;


    private double vision_reject_threshold_;

    private Pose2d vision_pose_ ;

    public SwerveVisionProcessing(SwerveBaseSubsystem sub, IVisionLocalization vision) throws BadParameterTypeException, MissingParameterException {
        vision_ = vision ;
        sub_ = sub ;

        vision_reject_threshold_ = sub_.getSettingsValue("estimator:vision-reject-threshold").getDouble();
        logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem("vision");
    }


    public Pose2d getCurrentPose() {
        Pose2d ret = null ;

        LocationData lc = vision_.getLocation(sub_.getPose());
        if (lc != null) { 
            ret = lc.location.toPose2d();
        }

        return ret;
    }

    public boolean hasTargets() {
        return vision_.getTagCount() > 0 ;
    }

    public void processVision() {
        MessageLogger logger = sub_.getRobot().getMessageLogger();

        LocationData lc = vision_.getLocation(sub_.getPose()) ;
        if (lc != null) {
            vision_pose_ = lc.location.toPose2d();

            double dist = vision_pose_.getTranslation().getDistance(sub_.getPose().getTranslation());
            if (dist < vision_reject_threshold_) {
                sub_.getEstimator().addVisionMeasurement(vision_pose_, lc.when) ;                 
            }
        }

        logger.startMessage(MessageType.Debug, logger_id_);
        logger.add("Vision: ");
        logger.add("dbx", sub_.getPose().getX());
        logger.add("dby", sub_.getPose().getY());
        logger.add("dbheading", sub_.getPose().getRotation().getDegrees());
        if (vision_pose_ != null) {
            logger.add("vsx", vision_pose_.getX());
            logger.add("vsy", vision_pose_.getY());
            logger.add("vsheading", vision_pose_.getRotation().getDegrees());
        }
        logger.endMessage();
    }

    public static Vector<N3> getParams(Subsystem sub, String str) throws BadParameterTypeException, MissingParameterException {
        double px = sub.getSettingsValue("estimator:" + str + ":x").getDouble();
        double py = sub.getSettingsValue("estimator:" + str + ":y").getDouble();
        double ph = sub.getSettingsValue("estimator:" + str + ":heading").getDouble();

        return VecBuilder.fill(px, py, ph);
    }

}
