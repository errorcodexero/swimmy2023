package frc.robot.subsystems.swerve;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveAction;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveLinearAlignAction extends SwerveDriveAction {
    private final int SampleCount = 10 ;
    private final static double AlignPValue =  0.35 ;
    private final static double AlignThreshold = 0.3 ;
    private LimeLightSubsystem ll_ ;

    private double[] samples_ ;
    private int total_ ;
    private int current_ ;

    public SwerveLinearAlignAction(SwerveBaseSubsystem sub, LimeLightSubsystem ll) {
        super(sub) ;

        ll_ = ll ;
        samples_ = new double[SampleCount] ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        current_ = 0 ;
        total_ = 0;
    }

    private boolean samplesOK() {
        boolean ret = true ;

        if (total_ < samples_.length)
            return false; 

        for(double v : samples_) {
            if (Math.abs(v) > AlignThreshold) {
                ret = false ;
                break ;
            }
        }

        return ret ;
    }

    @Override
    public void run() {
        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();

        double tx = ll_.getTX();

        samples_[current_++] = tx ;
        total_++ ;
        if (current_ == samples_.length) {
            current_ = 0  ;
        }

        if (samplesOK()) {
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
            logger.add("SwerveLinearAlign complete") ;
            logger.endMessage();

            getSubsystem().drive(new ChassisSpeeds());
            setDone() ;
        }
        else {
            double vel = -tx * AlignPValue ;
            ChassisSpeeds speeds = new ChassisSpeeds(0.0, vel, 0.0) ;
            getSubsystem().drive(speeds) ;

            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
            logger.add("SwerveLinearAlign aligning") ;
            logger.add("tx", tx) ;
            logger.add("vel", vel) ;
            for(int i = 0 ; i < samples_.length ; i++) {
                logger.add(Integer.toString(i), samples_[i]) ;
            }
            logger.endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveLinearAlignAction" ;
    }
}
