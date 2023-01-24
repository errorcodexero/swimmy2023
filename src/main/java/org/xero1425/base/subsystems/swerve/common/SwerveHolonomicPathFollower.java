package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
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

    private String pathname_ ;
    private XeroPath path_;
    private int index_ ;
    private boolean setpose_ ;

    private double start_ ;
    private int plot_id_ ;
    private Double[] plot_data_ ;

    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "ta (deg)",
        "ax (m)", "ay (m)", "aa (deg)"
    } ;

    public SwerveHolonomicPathFollower(SwerveBaseSubsystem sub, String pathname, boolean setpose) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        pathname_ = pathname ;
        setpose_ = setpose ;

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("holonomic" + pathname_) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().startPlot(plot_id_, columns_);

        start_ = getSubsystem().getRobot().getTime() ;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);

        if (setpose_) {
            Pose2d pose = getPoseFromPath(0) ;
            getSubsystem().setPose(pose);

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("SwerveHolonomicPathFollower: Initial Pose ", pose.toString()) ;
            logger.endMessage();
        }

        index_ = 0 ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {

        if (index_ < path_.getTrajectoryEntryCount())
        {
            Pose2d target = getPoseFromPath(index_);

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

            int i = 0 ;
            plot_data_[i++] = getSubsystem().getRobot().getTime() - start_ ;
            plot_data_[i++] = target.getX() ;
            plot_data_[i++] = target.getY() ;
            plot_data_[i++] = target.getRotation().getDegrees() ;
            plot_data_[i++] = actual.getX() ;
            plot_data_[i++] = actual.getY() ;
            plot_data_[i++] = actual.getRotation().getDegrees() ;
            getSubsystem().addPlotData(plot_id_, plot_data_) ;

            double velocity = getVelocityFromPath(index_) ;
            ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), target, velocity, target.getRotation()) ;
            getSubsystem().drive(speed) ;
            index_++ ;
        }

        if (index_ >= path_.getTrajectoryEntryCount())
        {
            getSubsystem().endPlot(plot_id_);
            getSubsystem().drive(new ChassisSpeeds()) ;
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveHolonomicPathFollower " + pathname_ ;
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
