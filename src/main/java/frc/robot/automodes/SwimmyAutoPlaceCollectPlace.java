package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyAutoPlaceCollectPlace extends SwimmyAutoMode  {
    public SwimmyAutoPlaceCollectPlace(AutoController ctrl, String side, String color, Location loc1, GamePiece what1,
            GridTagPosition grid2, Slot slot2, Location loc2, GamePiece what2) throws Exception {
        super(ctrl, color + "-" + side) ;

        //
        // Grab the loaded game piece and place it on the grid
        //
        grabAndPlace(loc1, what1);

        //
        // Drive a path across the charging station collecting on the other side
        //
        double beforedelay = ctrl.getRobot().getSettingsSupplier().get("automodes:left1:before-collect-delay").getDouble() ;
        driveAndCollect(side + color + "-Path1", true, beforedelay, Double.MAX_VALUE, what2);

        //
        // Drive back to place second cube
        //
        driveAndPlace(side + color + "-Path2", grid2, slot2, loc2, what2);
    }
}