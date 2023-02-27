package frc.robot.automodes;

import java.util.List;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyRobotAutoController extends AutoController {
    private SwimmyTestAutoMode test_mode_ ;
    
        public SwimmyRobotAutoController(XeroRobot robot) throws MissingParameterException, BadParameterTypeException {
            super(robot, "SwimmyRobotAutoController");
    
            try {
                test_mode_ = new SwimmyTestAutoMode(this) ;

                addAutoMode(new SwimmyAutoModeCenter1(this, "Red", Location.Top, GamePiece.Cone));
                addAutoMode(new SwimmyAutoModeCenter1(this, "Blue", Location.Top, GamePiece.Cone));
            }
            catch(Exception ex) {
                MessageLogger logger = robot.getMessageLogger() ;
                logger.startMessage(MessageType.Error).add("Exception thrown creating automodes - ") ;
                logger.add(ex.getMessage()).endMessage();
                robot.logStackTrace(ex.getStackTrace());
            }
        }
    
        public void updateAutoMode(int mode, String gamedata) {
    
            AutoMode modeobj = null ;
    
            if (isTestMode()) {
                modeobj = test_mode_ ;
            }
            else {
                List<AutoMode> automodes = getAllAutomodes() ;
                if (mode >= 0 && mode < automodes.size()) {

                    modeobj = automodes.get(mode) ;
                }
            }
    
            if (getAutoMode() != modeobj) {
                setAutoMode(modeobj) ;
            }
        }
}
