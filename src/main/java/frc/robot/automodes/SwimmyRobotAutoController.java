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
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyRobotAutoController extends AutoController {
    private SwimmyTestAutoMode test_mode_ ;
    
        public SwimmyRobotAutoController(XeroRobot robot) throws MissingParameterException, BadParameterTypeException {
            super(robot, "SwimmyRobotAutoController");
    
            try {
                test_mode_ = new SwimmyTestAutoMode(this) ;

                addAutoMode(new SwimmyAutoModeCenter1(this, "Red Balance", "Red", Location.Top, GamePiece.Cone));
                addAutoMode(new SwimmyAutoModeCenter1(this, "Blue Balance", "Blue", Location.Top, GamePiece.Cone));

                addAutoMode(new SwimmyAutoPlaceCollectPlace(this, "Red Two Center", "Middle1", "Red", Location.Top, GamePiece.Cone, GridTagPosition.Right, Slot.Left, Location.Top, GamePiece.Cone, 4.6));
                addAutoMode(new SwimmyAutoPlaceCollectPlace(this, "Blue Two Center", "Middle1", "Blue", Location.Top, GamePiece.Cone, GridTagPosition.Left, Slot.Right, Location.Top, GamePiece.Cone, 4.6));

                addAutoMode(new SwimmyAutoPlaceCollectPlace(this, "Red Two Edge", "Edge1", "Red", Location.Top, GamePiece.Cone, GridTagPosition.Left, Slot.Right, Location.Top, GamePiece.Cone, 4.9));
                addAutoMode(new SwimmyAutoPlaceCollectPlace(this, "Blue Two Edge", "Edge1", "Blue", Location.Top, GamePiece.Cone, GridTagPosition.Right, Slot.Left, Location.Top, GamePiece.Cone, 4.9));

                addAutoMode(new SwimmyThreeAutoMode(this, "Red Three", "Red", GamePiece.Cone, GamePiece.Cone, 4.6, 4.6)) ;
                addAutoMode(new SwimmyThreeAutoMode(this, "Blue Three", "Blue", GamePiece.Cone, GamePiece.Cone, 4.6, 4.6)) ;
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
