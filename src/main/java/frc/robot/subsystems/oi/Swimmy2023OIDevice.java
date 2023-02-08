package frc.robot.subsystems.oi;

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
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
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
    private int endgame_gadget_ ;
    private int turtle_gadget_ ;
    private int action_gadget_ ;

    private OILed state_output1_ ;
    private OILed state_output2_ ;

    private RobotOperation oper_ ;


    public Swimmy2023OIDevice(OISubsystem parent, String name, int index) throws BadParameterTypeException, MissingParameterException {
        super(parent, name, index);

        state_output1_.setState(State.OFF) ;
        state_output2_.setState(State.OFF) ;

        oper_ = new RobotOperation() ;
    }

    @Override
    public void createStaticActions() {
    }

    @Override
    public void generateActions() {

        RobotOperation oper = new RobotOperation(oper_) ;

        if (getValue(collect_v_place_gadget_) == 1) {
            oper_.setAction(Action.Collect) ;
        }
        else {
            oper_.setAction(Action.Place) ;
        }

        switch(getValue(cone_v_cube1_gadget_) | (getValue(cone_v_cube2_gadget_) << 1)) 
        {
            case 0:
                oper_.setGamePiece(GamePiece.Cone) ;
                break ;

            case 1:
                oper_.setGamePiece(GamePiece.None) ;
                break ;

            case 2:
                oper_.setGamePiece(GamePiece.Cube) ;
                break ;
        }

        switch(getValue(april_tag1_gadget_) | (getValue(april_tag2_gadget_) << 1)) 
        {
            case 0:
                oper_.setAprilTag(0) ;
                break ;

            case 1:
                oper_.setAprilTag(1) ;
                break ;

            case 2:
                oper_.setAprilTag(2) ;
                break ;
        }

        switch(getValue(slot_loc1_gadget_) | (getValue(slot_loc2_gadget_) << 1)) 
        {
            case 0:
                oper_.setSlot(Slot.Left) ;
                break ;

            case 1:
                oper_.setSlot(Slot.Middle) ;
                break ;

            case 2:
                oper_.setSlot(Slot.Right) ;
                break ;
        }

        switch(getValue(height1_gadget_) | (getValue(height2_gadget_) << 1)) 
        {
            case 0:
                oper_.setLocation(Location.Bottom);
                break ;

            case 1:
            oper_.setLocation(Location.Middle);
                break ;

            case 2:
            oper_.setLocation(Location.Top);
                break ;
        }

        if (getValue(abort_gadget_) == 1) {
            Swimmy2023RobotSubsystem sub = (Swimmy2023RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
            sub.abort() ;
        }
        else if (getValue(lock_gadget_) == 1) {
            Swimmy2023RobotSubsystem sub = (Swimmy2023RobotSubsystem)getSubsystem().getRobot().getRobotSubsystem();
            sub.setOperation(oper_) ;
            oper_ = new RobotOperation() ;
        }

        if (!oper.equals(oper_)) {
            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
            logger.add("Operation Changes: " + oper.toString() + " -> " + oper_.toString());
            logger.endMessage();
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

        num = getSubsystem().getSettingsValue("panel:gadgets:endgame").getInteger();
        endgame_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("panel:gadgets:turtle").getInteger();
        turtle_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ; 

        num = getSubsystem().getSettingsValue("panel:gadgets:action").getInteger();
        action_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;
    }
}
