package frc.robot.subsystems.swerve;

import org.xero1425.base.subsystems.Subsystem.DisplayType;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveBalancePlatform extends SwerveDriveAction {
    public enum XYDirection {
        XDirection,
        YDirection
    }

    private PIDCtrl pid_ ;
    private XYDirection dir_ ;

    public SwerveDriveBalancePlatform(SwerveBaseSubsystem sub, XYDirection dir) throws MissingParameterException, BadParameterTypeException {
        super(sub);

        dir_ = dir;
        String name = "subsystems:" + sub.getName() + ":balance-pid" ;
        pid_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), name, false);
    }

    @Override
    public void start() throws Exception {
        super.start() ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        double out ;
        ChassisSpeeds speed ;
        double value = 0.0 ;

        if (dir_ == XYDirection.XDirection) {
            value = getSubsystem().getPitch() ;
            out = pid_.getOutput(0.0, value, getSubsystem().getRobot().getDeltaTime());
            speed = new ChassisSpeeds(out, 0.0, 0.0);
        }
        else {
            value = getSubsystem().getRoll() ;
            out = pid_.getOutput(0.0, value, getSubsystem().getRobot().getDeltaTime());
            speed = new ChassisSpeeds(0.0, out, 0.0);
        }

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug);
        logger.add("AutoBalance: ") ;
        logger.add("angle", value) ;
        logger.add("power", out) ;
        logger.endMessage();

        getSubsystem().putDashboard("balance", DisplayType.Always, out);
        getSubsystem().drive(speed);
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveBalancePlatform";
    }
}
