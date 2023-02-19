package frc.robot.subsystems.swerve;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveBalancePlatform extends SwerveDriveAction {

    private PIDCtrl pid_ ;

    public SwerveDriveBalancePlatform(SwerveBaseSubsystem sub) throws MissingParameterException, BadParameterTypeException {
        super(sub);

        pid_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), "balance-pid", false);
    }

    @Override
    public void start() throws Exception {
        super.start() ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        double out = pid_.getOutput(0.0, getSubsystem().getPitch(), getSubsystem().getRobot().getDeltaTime());
        ChassisSpeeds speeds = new ChassisSpeeds(out, 0.0, 0.0);
        getSubsystem().drive(speeds);
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveBalancePlatform";
    }
}
