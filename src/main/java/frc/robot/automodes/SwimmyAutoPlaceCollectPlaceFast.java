package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.ISettingsSupplier;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyAutoPlaceCollectPlaceFast extends SwimmyAutoMode  {
    public SwimmyAutoPlaceCollectPlaceFast(AutoController ctrl, String name, String side, String color, GridTagPosition tag2, Slot slot2) throws Exception {

        super(ctrl, name) ;

        ISettingsSupplier settings = ctrl.getRobot().getSettingsSupplier() ;
        String settingsprefix = "automodes:" + side + ":" + color + ":" ;                           // e.g. automodes:middle1fast:red:
        String pathprefix = side + color + "-" ;                                                    // e.g. middle1fastred-

        //
        // Get things from the settings file
        // 

        //
        // Start collect distance for path 1
        //
        double sc1 = settings.get(settingsprefix + "1:start-collect").getDouble() ;

        //
        // Close collect distance for path 1
        //
        double cc1 = settings.get(settingsprefix + "1:start-collect").getDouble() ;

        //
        // Place operation distance for path 1
        //
        double sp1 = settings.get(settingsprefix + "1:start-place").getDouble() ;

        //
        // Start collect distance for path 2
        //
        double sc2 = settings.get(settingsprefix + "2:start-collect").getDouble() ;

        //
        // Close collect distance for path 2
        //
        double cc2 = settings.get(settingsprefix + "2:start-collect").getDouble() ;        

        //
        // Grab the loaded game piece and place it on the grid
        //
        grabAndPlace(Location.Top, GamePiece.Cone) ;

        //
        // Drive a path across the charging station collecting on the other side
        //
        driveAndCollectAndPlace(pathprefix + "Path1", true, sc1, cc1, sp1, GamePiece.Cone, tag2, slot2, Location.Top) ;

        //
        // Draw and collect
        //
        driveAndCollect2(pathprefix + "Path2", true, sc2, cc2, GamePiece.Cone) ;
    }
}
