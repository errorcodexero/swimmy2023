package frc.robot.automodes;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.SwimmyRobot2023;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoModeCenter1 extends SwimmyAutoMode {

    public SwimmyAutoModeCenter1(AutoController ctrl, Location location, GamePiece piece) throws InvalidActionRequest, BadParameterTypeException, MissingParameterException {
        super(ctrl, "center1");

        var robot = (Swimmy2023RobotSubsystem)ctrl.getRobot().getRobotSubsystem();
        var gpm = robot.getGPM();

        var color = DriverStation.getAlliance() == Alliance.Red ? "red" : "blue";

        grabAndPlace(location, piece);

        drivePath("center1-"+color+"1", true, 5);

        drivePath("center1-"+color+"2", true, 5);

        addSubActionPair(robot_.getSwerve(), new SwerveDriveBalancePlatform(robot.getSwerve()), true);

    }
    
}
