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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveVisionProcessing {
    private enum VisionParamsType {
        SingleNear,
        SingleFar,
        MultiNear,
        MultiFar
    };

    private int logger_id_ ;

    private SwerveBaseSubsystem sub_ ;
    private IVisionLocalization vision_ ;

    private VisionParamsType params_type_ ;

    private double single_tag_threshold_;
    private Vector<N3> single_tag_near_params_ ;
    private Vector<N3> single_tag_far_params_ ;

    private double multi_tag_threshold_;
    private Vector<N3> multi_tag_near_params_ ;
    private Vector<N3> multi_tag_far_params_ ;

    private double vision_reject_threshold_;

    private boolean added_ ;
    private Pose2d vision_pose_ ;

    public SwerveVisionProcessing(SwerveBaseSubsystem sub, IVisionLocalization vision) throws BadParameterTypeException, MissingParameterException {
        vision_ = vision ;
        sub_ = sub ;

        single_tag_threshold_ = sub_.getSettingsValue("estimator:single-threshold").getDouble();
        multi_tag_threshold_ = sub_.getSettingsValue("estimator:single-threshold").getDouble();
        vision_reject_threshold_ = sub_.getSettingsValue("estimator:vision-reject-threshold").getDouble();

        single_tag_near_params_ = getParams(sub, "vision:single-near");
        single_tag_far_params_ = getParams(sub, "vision:single-far");
        multi_tag_near_params_ = getParams(sub, "vision:multi-near");
        multi_tag_far_params_ = getParams(sub, "vision:multi-far");

        added_ = false ;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision");
        shuffleboardTab.addNumber("DB Heading", () -> sub_.getHeading().getDegrees());
        shuffleboardTab.addNumber("DB X", () -> sub_.getPose().getX());
        shuffleboardTab.addNumber("DB Y", () -> sub_.getPose().getY());

        shuffleboardTab.addNumber("VS Heading", () -> getVisionDegrees());
        shuffleboardTab.addNumber("VS X", () -> getVisionX());
        shuffleboardTab.addNumber("VS Y", () -> getVisionY());

        shuffleboardTab.addBoolean("Adding", () -> { return added_ ;});
        shuffleboardTab.addString("Params", () -> { return params_type_.toString() ;});

        logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem("vision");
    }

    private double getVisionDegrees() {
        if (vision_pose_ == null) {
            return Double.NaN;
        }

        return vision_pose_.getRotation().getDegrees();
    }

    private double getVisionX() {
        if (vision_pose_ == null) {
            return Double.NaN;
        }

        return vision_pose_.getX();
    }

    private double getVisionY() {
        if (vision_pose_ == null) {
            return Double.NaN;
        }

        return vision_pose_.getY();
    }

    public void processVision() {
        MessageLogger logger = sub_.getRobot().getMessageLogger();

        added_ = false ;
        LocationData lc = vision_.getLocation() ;
        setVisionParams();
        if (lc != null) {
            vision_pose_ = new Pose2d(lc.location.getX(), lc.location.getZ(), new Rotation2d(lc.location.getRotation().getY()));
            vision_pose_ = lc.location.toPose2d();
            double dist = vision_pose_.getTranslation().getDistance(sub_.getPose().getTranslation());
            if (dist < vision_reject_threshold_) {
                sub_.getEstimator().addVisionMeasurement(vision_pose_, lc.when) ; 
                added_ = true ;
            }
            else {
                logger.startMessage(MessageType.Warning);
                logger.add("Ignoring vision sample");
                logger.add("dbpose", sub_.getPose());
                logger.add("vision", vision_pose_);
                logger.add("dist", dist);
                logger.endMessage(); 
            }
        }      
        

        logger.startMessage(MessageType.Debug, logger_id_);
        logger.add("Vision: ");
        logger.add("params", params_type_.toString());
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

    private void setVisionParams(VisionParamsType vtype)
    {
        MessageLogger logger = sub_.getRobot().getMessageLogger();
        if (params_type_ != vtype) {
            switch(vtype) {
                case SingleNear:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Single Near").endMessage();
                    sub_.getEstimator().setVisionMeasurementStdDevs(single_tag_near_params_);
                    break ;

                case SingleFar:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Single Far").endMessage();
                    sub_.getEstimator().setVisionMeasurementStdDevs(single_tag_far_params_);
                    break; 

                case MultiNear:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Multi Near").endMessage();
                    sub_.getEstimator().setVisionMeasurementStdDevs(multi_tag_near_params_);
                    break ;

                case MultiFar:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Multi Far").endMessage();                
                    sub_.getEstimator().setVisionMeasurementStdDevs(multi_tag_far_params_);
                    break; 
            }
            params_type_ = vtype;
        }
    }
    

    private void setVisionParams() {
        if (vision_.getTagCount() == 1) {
            if (vision_.getDistance() < single_tag_threshold_) {
                setVisionParams(VisionParamsType.SingleNear);
            }
            else {
                setVisionParams(VisionParamsType.SingleFar);
            }
        }
        else {
            if (vision_.getDistance() < multi_tag_threshold_) {
                setVisionParams(VisionParamsType.MultiNear);
            }
            else {
                setVisionParams(VisionParamsType.MultiFar);
            }                
        }
    }

    public static Vector<N3> getParams(Subsystem sub, String str) throws BadParameterTypeException, MissingParameterException {
        double px = sub.getSettingsValue("estimator:" + str + ":x").getDouble();
        double py = sub.getSettingsValue("estimator:" + str + ":y").getDouble();
        double ph = sub.getSettingsValue("estimator:" + str + ":heading").getDouble();

        return VecBuilder.fill(px, py, ph);
    }

}
