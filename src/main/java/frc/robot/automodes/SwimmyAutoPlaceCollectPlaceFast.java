package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyAutoPlaceCollectPlaceFast extends SwimmyAutoMode  {
    public SwimmyAutoPlaceCollectPlaceFast(AutoController ctrl, String name, String side, String color, Location loc1, GamePiece what1,
            GridTagPosition grid2, Slot slot2, Location loc2, GamePiece what2, double lambdist) throws Exception {
        super(ctrl, name) ;

        //
        // Grab the loaded game piece and place it on the grid
        //
        grabAndPlace(loc1, what1);

        //
        // Drive a path across the charging station collecting on the other side
        //
        double beforedelay = ctrl.getRobot().getSettingsSupplier().get("automodes:" + side + ":before-collect-delay").getDouble() ;
        double grabdelay = ctrl.getRobot().getSettingsSupplier().get("automodes:" + side + ":grab-delay").getDouble() ;
        driveAndCollectAndPlace(side + color + "-Path1", true, beforedelay, grabdelay, what2, lambdist, grid2, slot2, loc2) ;
    }
}
