package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveAction;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveSteerToTarget extends SwerveDriveAction {
    private Supplier<Boolean> isAtTarget_ ;
    private LimeLightSubsystem limelight_ ;
    private boolean timer_running_ ;
    private XeroTimer timer_ ;
    private double kp_ ;

    public SwerveDriveSteerToTarget(LimeLightSubsystem ll, SwerveBaseSubsystem swerve, Supplier<Boolean> isAtTarget) {
        super(swerve);

        isAtTarget_ = isAtTarget ;
        limelight_ = ll ;
        timer_ = new XeroTimer(swerve.getRobot(), "SwerveDriveSteerToTarget", 2.0);
        kp_ = 1.0 ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        timer_running_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (isAtTarget_.get()) {
            //
            // We have the game piece, 
            //
            getSubsystem().drive(new ChassisSpeeds());
            setDone() ;
        }
        else 
        {
            if (timer_running_ == false) {
                if (limelight_.isTargetDetected()) {
                    //
                    // Steer left and right
                    //
                    double forward = 1.0 ;
                    double leftright = limelight_.getTX() * kp_ ;
                    ChassisSpeeds speeds = new ChassisSpeeds(forward, leftright, 0.0) ;
                    getSubsystem().drive(speeds);
                }
                else
                {
                    //
                    // Keep driving straight
                    //
                    timer_running_ = true ;
                    timer_.start();
                }
            }
            else 
            {
                if (timer_.isExpired()) {
                    getSubsystem().drive(new ChassisSpeeds());
                    setDone() ;
                }
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveSteerToTarget" ;
    }
}
