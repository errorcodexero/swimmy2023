package frc.robot.subsystems.swerve;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveAction;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.PIDCtrl;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveAlignRobotAction extends SwerveDriveAction {
    private LimeLightSubsystem ll_ ;
    private PIDCtrl ctrl_ ;
    private double threshold_ ;
    private XeroTimer start_timer_ ;

    public SwerveAlignRobotAction(SwerveBaseSubsystem sub, LimeLightSubsystem ll) {
        super(sub) ;

        ll_ = ll ;
        threshold_ = 3.0 ;

        double p = 0.2 ;
        double i = 0 ;
        double d = 0 ;
        double f = 0 ;
        double minout = -1.0 ;
        double maxout = 1.0 ;
        double maxint = 0.0 ;
        ctrl_ = new PIDCtrl(p, i, d, f, minout, maxout, maxint, false) ;

        start_timer_ = new XeroTimer(sub.getRobot(), "startimer", 0.75);
    }

    @Override
    public void start() throws Exception {
        super.start();
        ll_.setPipeline(1);
        start_timer_.start() ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();

        if (ll_.isTargetDetected()) {
            if (Math.abs(ll_.getTX()) < threshold_) {
                logger.startMessage(MessageType.Debug);
                logger.add("Alignment achieved - action done");
                logger.add("tx", ll_.getTX());
                logger.endMessage() ;
                setDone() ;
                getSubsystem().drive(new ChassisSpeeds());
            }
            else {
                double out = ctrl_.getOutput(0.0, ll_.getTX(), getSubsystem().getRobot().getDeltaTime());
                ChassisSpeeds speed = new ChassisSpeeds(0.0, out, 0.0) ;
                getSubsystem().drive(speed) ;

                start_timer_ = null ;

                logger.startMessage(MessageType.Debug);
                logger.add("Align");
                logger.add("target", ll_.isTargetDetected());
                logger.add("tx", ll_.getTX());
                logger.add("speed", out);
                logger.endMessage();
            }
        }
        else {
            if (start_timer_ != null && start_timer_.isExpired()) {
                logger.startMessage(MessageType.Debug);
                logger.add("Align action failed waiting on pipeline");
                logger.endMessage() ;
                setDone() ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveAlignRobot" ;
    }
}
