package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;

import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform.XYDirection;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoModeCenter1 extends SwimmyAutoMode  {
    public SwimmyAutoModeCenter1(AutoController ctrl, String color, Location where, GamePiece what) throws Exception {
        super(ctrl, color + "Center-1") ;

        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        //
        // Grab the loaded game piece and place it on the grid
        //
        grabAndPlace(where, what);

        //
        // Drive a path across the charging station collecting on the other side
        //
        double beforedelay = ctrl.getRobot().getSettingsSupplier().get("automodes:center1:before-collect-delay").getDouble() ;
        double collectdelay = ctrl.getRobot().getSettingsSupplier().get("automodes:center:collect-delay").getDouble();
        driveAndCollect("Center1" + color + "-Path1", true, beforedelay, collectdelay, RobotOperation.GamePiece.Cone);

        //
        // Drive to the center of the platform rotating 90 degrees
        //
        addSubActionPair(robot.getSwerve(), new SwerveHolonomicPathFollower(robot.getSwerve(), "Center1" + color + "-Path2", false, 1.0) , true);

        //
        // Delay for 1 second to be sure the platform has settled
        //
        addAction(new DelayAction(getAutoController().getRobot(), 1.0));

        //
        // Now run the auto balance operation (with the robot sideways)
        //
        addSubActionPair(robot.getSwerve(), new SwerveDriveBalancePlatform(robot.getSwerve(), XYDirection.YDirection), true);
    }
}
