package frc.robot.automodes;

import org.xero1425.base.actions.DispatchAction;
import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.gpm.GPMShootAction;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;

public class SwimmyThreeAutoMode extends SwimmyAutoMode  {
    public SwimmyThreeAutoMode(AutoController ctrl, String name, String color, GamePiece what2, GamePiece what3, double lam2, double lam3) throws Exception {
        super(ctrl, name) ;

        double beforedelay1 = ctrl.getRobot().getSettingsSupplier().get("automodes:three:" + color + ":path1:before-collect-delay").getDouble() ;
        double grabdelay1 = ctrl.getRobot().getSettingsSupplier().get("automodes:three:" + color + ":path1:grab-delay").getDouble() ;
        double beforedelay2 = ctrl.getRobot().getSettingsSupplier().get("automodes:three:" + color + ":path2:before-collect-delay").getDouble() ;
        double grabdelay2 = ctrl.getRobot().getSettingsSupplier().get("automodes:three:" + color + ":path2:grab-delay").getDouble() ;

        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();
        GPMShootAction shoot = new GPMShootAction(robot.getGPM()) ;
        DispatchAction da = new DispatchAction(robot.getGPM(), shoot, true) ;

        //
        // In parallel, drive the first path and spit out the cone and then go collect a second cone
        //
        driveAndCollect("Three" + color + "-Path1", true, beforedelay1, grabdelay1, what2, lam2, da) ;

        //
        // Drive the second path back to the grid
        //
        drivePath("Three" + color + "-Path2", false);

        //
        // In parallel, drive the third path back out to collect a new cone, spitting out the cone we are holding
        //
        shoot = new GPMShootAction(robot.getGPM()) ;
        da = new DispatchAction(robot.getGPM(), shoot, true) ;
        driveAndCollect("Three" + color + "-Path3", true, beforedelay2, grabdelay2, what3, lam3, da) ;

        //
        // Drive the fourth path back to the grid
        // 
        drivePath("Three" + color + "-Path4", false);

        //
        // And shoot the final cone
        //
        shoot = new GPMShootAction(robot.getGPM()) ;
        addSubActionPair(robot.getGPM(), shoot, true) ;
    }
}
