package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.plotting.PlotDataSource;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveHolonomicPathFollower extends SwerveHolonomicControllerAction {

    public interface todoLambda {
        void doit() ;
    }

    private String pathname_ ;
    private XeroPath path_;
    private int index_ ;
    private boolean setpose_ ;

    private double start_ ;
    private int plot_id_ ;
    private PlotDataSource plot_src_ ;

    private boolean end_phase_;
    private XeroTimer end_timer_;

    private boolean disable_vision_ ;

    private todoLambda lambda_ ;
    private double distance_ ;



    public SwerveHolonomicPathFollower(SwerveBaseSubsystem sub, String pathname, boolean setpose, double endtime) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        pathname_ = pathname ;
        setpose_ = setpose ;


        plot_id_ = getSubsystem().initPlot(pathname_) ;
        createPlotDataSource();

        end_timer_ = new XeroTimer(sub.getRobot(), "holonomicpath", endtime);
        disable_vision_ = true ;
    }

    private void createPlotDataSource() {
        plot_src_ = new PlotDataSource() ;

        plot_src_.addDataElement("time", () -> { return getSubsystem().getRobot().getTime() - start_ ;});

        plot_src_.addDataElement("tx (m)", () -> { return getPoseFromPath(index_).getX() ; });
        plot_src_.addDataElement("ty (m)", () -> { return getPoseFromPath(index_).getY() ; });
        plot_src_.addDataElement("ta (deg)", () -> { return getPoseFromPath(index_).getRotation().getDegrees() ; });

        plot_src_.addDataElement("ax (m)", () -> { return getSubsystem().getPose().getX() ; });
        plot_src_.addDataElement("ay (m)", () -> { return getSubsystem().getPose().getY() ; });
        plot_src_.addDataElement("aa (deg)", () -> { return getSubsystem().getPose().getRotation().getDegrees() ; });
    }

    public void setLambda(todoLambda lambda, double dist) {
        lambda_ = lambda ;
        distance_ = dist ;
    }

    public String getPathName() {
        return pathname_ ;
    }

    public void disableVision(boolean b) {
        disable_vision_ = b ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (disable_vision_) {
            getSubsystem().enableVision(false);
        }
        getSubsystem().startPlot(plot_id_, plot_src_);

        start_ = getSubsystem().getRobot().getTime() ;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);

        end_phase_ = false ;

        if (setpose_) {
            Pose2d pose = getPoseFromPath(0) ;
            getSubsystem().setPose(pose);

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("SwerveHolonomicPathFollower: Initial Pose ", getSubsystem().getPose());
            logger.endMessage();
        }

        index_ = 0 ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        double velocity ;
        Pose2d target ;

        if (index_ < path_.getTrajectoryEntryCount())
        {
            target = getPoseFromPath(index_);
            velocity = getVelocityFromPath(index_) ;
            
            double dist = getDistanceFromPath(index_);
            if (lambda_ != null && dist > distance_) {
                MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Info);
                logger.add("PathFollowing executing lambda") ;
                logger.add("target", distance_);
                logger.add("actual", dist) ;
                logger.endMessage();

                lambda_.doit() ;
                lambda_ = null ;
            }
        }
        else {
            target = getPoseFromPath(index_ - 1);
            velocity = 0.0 ;
        }


        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), target, velocity, target.getRotation()) ;
        getSubsystem().drive(speed) ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("SwerveHolonomicPathFollower Target:").add("index", index_) ;
        logger.add(", target ") ;
        logger.add(target.getX()).add(" ").add(target.getY()) ;
        logger.add(" ").add(target.getRotation().getDegrees()) ;

        Pose2d actual = getSubsystem().getPose() ;
        logger.add(",actual ") ;
        logger.add(actual.getX()).add(" ").add(actual.getY()) ;
        logger.add(" ").add(actual.getRotation().getDegrees()) ;

        logger.endMessage();
        getSubsystem().addPlotData(plot_id_);
        
        if (index_ < path_.getTrajectoryEntryCount()) {
            index_++ ;            
        }
        else {
            if (!end_phase_) {
                end_phase_ = true ;
                end_timer_.start() ;
            }

            if (index_ == path_.getTrajectoryEntryCount() && (controller().atReference() || end_timer_.isExpired())) {
                getSubsystem().endPlot(plot_id_);
                getSubsystem().drive(new ChassisSpeeds()) ;
                getSubsystem().enableVision(true);
                setDone();

                logger.startMessage(MessageType.Info) ;
                logger.add("finished path") ;
                logger.addQuoted(pathname_);
                logger.add("pose", actual);
                logger.endMessage();
            }
        }
    }

    @Override
    public void cancel() {
        System.out.println("me") ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveHolonomicPathFollower " + pathname_ ;
    }

    private double getDistanceFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return main.getPosition();
    }

    private Pose2d getPoseFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return new Pose2d(main.getX(), main.getY(), Rotation2d.fromDegrees(main.getRotation())) ;
    }

    private double getVelocityFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return main.getVelocity() ;
    }
}
