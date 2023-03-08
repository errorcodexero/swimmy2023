package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform.XYDirection;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoModeCenter1 extends SwimmyAutoMode  {
    public SwimmyAutoModeCenter1(AutoController ctrl, String name, String color, Location where, GamePiece what) throws Exception {
        super(ctrl, name) ;

        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        //
        // Grab the loaded game piece and place it on the grid
        //
        grabAndPlace(where, what);

        //
        // Drive a path across the charging station collecting on the other side
        //
        drivePath("Center1" + color + "-Path1", true) ;

        drivePath("Center1" + color + "-Path2", false) ;

        //
        // Now run the auto balance operation (with the robot sideways)
        //
        addSubActionPair(robot.getSwerve(), new SwerveDriveBalancePlatform(robot.getSwerve(), XYDirection.XDirection), true);
    }
}
