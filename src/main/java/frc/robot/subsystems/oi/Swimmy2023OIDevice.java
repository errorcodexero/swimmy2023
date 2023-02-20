package frc.robot.subsystems.oi;

//
// Switches, purpose, and types
//
// Auto vs Manual       Single Toggle (1 DIO)
// Collect vs Place     Single Toggle (1 DIO)
// Cone vs Cube         Single Toggle (1 DIO)
// Lock                 Button (1 DIO)
// Abort                Button (1 DIO)
// April Tag            Three way toggle (2 DIOs)
// Slot                 Three way toggle (2 DIOs)
// Height               Three way toggle (2 DIOs)
// EndGame              Single Toggle (1 DIO)
// Turtle               Button (1 DIO)
// Action               Button (1 DIO)


import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OIPanelButton;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OILed.State;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.TurtleAction;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class Swimmy2023OIDevice extends OIPanel {
    private int collect_v_place_gadget_ ;
    private int cone_v_cube1_gadget_ ;
    private int cone_v_cube2_gadget_ ;
    private int auto_v_manual_gadget_ ;
    private int lock_gadget_ ;
    private int abort_gadget_ ;
    private int april_tag1_gadget_ ;
    private int april_tag2_gadget_ ;
    private int slot_loc1_gadget_ ;
    private int slot_loc2_gadget_ ;
    private int height1_gadget_ ;
    private int height2_gadget_ ;
    private int turtle_gadget_ ;
    private int action_gadget_ ;
    private int drop_gadget_;

    private OILed state_output1_ ;
    private OILed state_output2_ ;

    private RobotOperation oper_ ;

    private TurtleAction turtle_action_ ;

    public Swimmy2023OIDevice(OISubsystem parent, String name, int index) throws BadParameterTypeException, MissingParameterException {
        super(parent, name, index);

        state_output1_.setState(State.OFF) ;
        state_output2_.setState(State.OFF) ;

        oper_ = new RobotOperation() ;
    }

    public boolean isActionButtonPressed() {
        return getValue(action_gadget_) == 1 ;
    }

    public boolean isDropButtonPressed() {
        return getValue(drop_gadget_) == 1;
    }

    @Override
    public void createStaticActions() throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem() ;

        turtle_action_ = new TurtleAction(robot) ;
    }

    private RobotOperation extractRobotOperation() {
        RobotOperation oper = new RobotOperation() ; 

        if (getValue(auto_v_manual_gadget_) == 0) {
            oper.setManual(true);
        }
        
        if (getValue(collect_v_place_gadget_) == 0) {
            oper.setAction(Action.Collect) ;
        }
        else {
            oper.setAction(Action.Place) ;
        }

        switch(getValue(cone_v_cube1_gadget_) | (getValue(cone_v_cube2_gadget_) << 1)) 
        {
            case 0:
                oper.setGamePiece(GamePiece.None) ;
                break ;

            case 1:
                oper.setGamePiece(GamePiece.Cube) ;
                break ;

            case 2:
                oper.setGamePiece(GamePiece.Cone) ;
                break ;
        }

        switch(getValue(april_tag1_gadget_) | (getValue(april_tag2_gadget_) << 1)) 
        {
            case 0:
                oper.setAprilTag(GridTagPosition.Left) ;
                break ;

            case 1:
                oper.setAprilTag(GridTagPosition.Middle) ;
                break ;

            case 2:
                oper.setAprilTag(GridTagPosition.Right) ;
                break ;
        }

        switch(getValue(slot_loc1_gadget_) | (getValue(slot_loc2_gadget_) << 1)) 
        {
            case 0:
                oper.setSlot(Slot.Middle) ;
                break ;

            case 1:
                oper.setSlot(Slot.Left) ;
                break ;

            case 2:
                oper.setSlot(Slot.Right) ;
                break ;
        }

        switch(getValue(height1_gadget_) | (getValue(height2_gadget_) << 1)) 
        {
            case 0:
                oper.setLocation(Location.Bottom);
                break ;

            case 1:
                oper.setLocation(Location.Middle);
                break ;

            case 2:
                oper.setLocation(Location.Top);
                break ;
        }

        return oper ;
    }


    @Override
    public void generateActions() {
        super.generateActions();

        Swimmy2023RobotSubsystem sub = (Swimmy2023RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
        boolean errDetected = false ;
        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();

        if (getValue(turtle_gadget_) == 1) {
            //
            // First priority is the turtle action.  This moves everything back into the robot to the best of our
            // ability.  It will cancel any running actions with the arm and grabber.
            //
            sub.setAction(turtle_action_);
            setLeds(false) ;
        }
        else if (getValue(abort_gadget_) == 1) {
            //
            // Abort any action going in the robot subsystem
            //
            sub.abort() ;
            setLeds(false) ;
        }
        else {
            RobotOperation oper = extractRobotOperation() ;
            if (!oper.equals(oper_)) {
                logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
                logger.add("OI operation changed: ");
                logger.add(oper_.toString());
                logger.add(" -> ");
                logger.add(oper.toString());
                logger.endMessage();
            }

            oper_ = oper;
            if (getValue(action_gadget_) == 1&& !oper_.getManual()) {
                //
                // We are in automatic mode and the action button has been
                // pressed.  For now, this is an override for ground pickup
                //
                oper_.setGround(true);
                errDetected = sub.setOperation(oper_);
            }
            else if (getValue(lock_gadget_) == 1) {
                //
                // The lock button has been pressed, lock in the gunners request
                //
                if (oper_.getManual() == false) {

                    errDetected = sub.setOperation(oper_);

                    logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
                    logger.add("OI assigned op (auto): " + oper_.toString());
                    logger.endMessage();
                    
                } else {
                    errDetected = sub.setOperation(oper_);

                    logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
                    logger.add("OI assigned op (manual): " + oper_.toString());
                    logger.endMessage();
                }
            }

            setLeds(errDetected) ;
        }
    }

    private void setLeds(boolean err) {
        if (err) {
            state_output1_.setState(State.ON);
            state_output2_.setState(State.ON);
        }
        else if (oper_.getGamePiece() == GamePiece.Cone) {
            state_output1_.setState(State.ON);
            state_output2_.setState(State.OFF);
        }
        else if (oper_.getGamePiece() == GamePiece.Cube) {
            state_output1_.setState(State.OFF);
            state_output2_.setState(State.ON);
        }
        else {
            state_output1_.setState(State.OFF);
            state_output2_.setState(State.OFF);
        }
    }

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();

        state_output1_ = createLED(getSubsystem().getSettingsValue("panel:outputs:state1").getInteger()) ;
        state_output2_ = createLED(getSubsystem().getSettingsValue("panel:outputs:state2").getInteger()) ;        
    }

    @Override
    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        super.initializeGadgets();
        int num ;

        num = getSubsystem().getSettingsValue("panel:gadgets:collect_v_place").getInteger();
        collect_v_place_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:cone_v_cube:1").getInteger();
        cone_v_cube1_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;      

        num = getSubsystem().getSettingsValue("panel:gadgets:cone_v_cube:2").getInteger();
        cone_v_cube2_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;         

        num = getSubsystem().getSettingsValue("panel:gadgets:auto_v_manual").getInteger();
        auto_v_manual_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:lock").getInteger();
        lock_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger();
        abort_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:apriltag:1").getInteger();
        april_tag1_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:apriltag:2").getInteger();
        april_tag2_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;       
        
        num = getSubsystem().getSettingsValue("panel:gadgets:slot:1").getInteger();
        slot_loc1_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:slot:2").getInteger();
        slot_loc2_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;     
        
        num = getSubsystem().getSettingsValue("panel:gadgets:height:1").getInteger();
        height1_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;
        
        num = getSubsystem().getSettingsValue("panel:gadgets:height:2").getInteger();
        height2_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;  

        num = getSubsystem().getSettingsValue("panel:gadgets:turtle").getInteger();
        turtle_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ; 

        num = getSubsystem().getSettingsValue("panel:gadgets:action").getInteger();
        action_gadget_ = mapButton(num, OIPanelButton.ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:action").getInteger();
        action_gadget_ = mapButton(num, OIPanelButton.ButtonType.LowToHigh) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:drop").getInteger();
        drop_gadget_ = mapButton(num, OIPanelButton.ButtonType.LowToHigh) ;
    }
}
