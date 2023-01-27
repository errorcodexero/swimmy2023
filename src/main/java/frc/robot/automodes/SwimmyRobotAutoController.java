package frc.robot.automodes;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;

public class SwimmyRobotAutoController extends AutoController {
    public SwimmyRobotAutoController(XeroRobot robot) throws Exception {
        super(robot, "SwimmyRobotAutoController");

        setTestMode(new SwimmyTestAutoMode(this)) ;
        
        addAutoMode(new SwimmyAutoModeLeft(this));
        addAutoMode(new SwimmyAutoModeMiddle(this));
        addAutoMode(new SwimmyAutoModeRight(this));
    }
}
