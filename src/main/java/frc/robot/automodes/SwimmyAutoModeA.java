package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoModeA extends SwimmyAutoMode {

    public SwimmyAutoModeA(AutoController ctrl, Location location, GamePiece piece) throws Exception {
        super(ctrl, "a");

        var robot = (Swimmy2023RobotSubsystem)ctrl.getRobot().getRobotSubsystem();
        var gpm = robot.getGPM();

        var color = DriverStation.getAlliance() == Alliance.Red ? "red" : "blue";

        grabAndPlace(location, piece);

        drivePath("a-"+color+"1", true, 5);

        addSubActionPair(gpm, new GPMCollectAction(gpm, true), true);

    }
    
}
