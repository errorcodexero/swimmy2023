package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoModeOneEdge extends SwimmyAutoMode {
    public SwimmyAutoModeOneEdge(AutoController ctrl, String name, String color, Location loc1, GamePiece what1) throws Exception {
        super(ctrl, name) ;

        //
        // Grab the loaded game piece and place it on the grid
        //
        grabAndPlace(loc1, what1);

        drivePath("Edge1" + color + "-Path1", false) ;
    }
}
