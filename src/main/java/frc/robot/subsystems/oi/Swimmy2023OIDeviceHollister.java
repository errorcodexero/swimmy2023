package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OILed.State;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.SwimmyRobot2023;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.TurtleAction;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

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
//

public class Swimmy2023OIDeviceHollister extends OIPanel {
    private int collect_v_place_gadget_;
    private int auto_v_manual_gadget_;
    private int lock_gadget_;
    private int abort_gadget_;
    private int turtle_gadget_;
    private int action_gadget_;
    private int drop_gadget_;

    private int cone_v_cube_1_gadget_;
    private int cone_v_cube_2_gadget_;

    private int apriltag_1_gadget_; 
    private int apriltag_2_gadget_;
    
    private int slot_1_gadget_; 
    private int slot_2_gadget_;
    
    private int height_1_gadget_; 
    private int height_2_gadget_;

    private OILed state_output1_ ;
    private OILed state_output2_ ;


    private org.xero1425.base.actions.Action turtleAction;

    public Swimmy2023OIDeviceHollister(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);
    }

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }
       
    public boolean isActionButtonPressed() {
        return getValue(action_gadget_) == 1 ;
    }

    public boolean isDropButtonPressed() {
        return getValue(drop_gadget_) == 1;
    }

    @Override
    public void createStaticActions() throws MissingParameterException, BadParameterTypeException {
        SwimmyRobot2023 robot = (SwimmyRobot2023)getSubsystem().getRobot();
        Swimmy2023RobotSubsystem subsystem = (Swimmy2023RobotSubsystem)robot.getRobotSubsystem();
        turtleAction = new TurtleAction(subsystem);
    }

    private GamePiece getGamePiece() {
        if (getValue(cone_v_cube_1_gadget_) == 0 && getValue(cone_v_cube_2_gadget_) == 1) {
            return GamePiece.Cone;
        } else if (getValue(cone_v_cube_1_gadget_) == 1 && getValue(cone_v_cube_2_gadget_) == 0) {
            return GamePiece.Cube;
        } else {
            return GamePiece.None;
        }
    }

    private RobotOperation.GridTagPosition getTag() {
        if (getValue(apriltag_1_gadget_) == 0 && getValue(apriltag_2_gadget_) == 0) {
            return GridTagPosition.Middle;
        } else if (getValue(apriltag_1_gadget_) == 1 && getValue(apriltag_2_gadget_) == 0) {
            return GridTagPosition.Left;
        } else {
            return GridTagPosition.Right;
        }
    }
    
    private RobotOperation.Slot getSlot() {
        if (getValue(slot_1_gadget_) == 0 && getValue(slot_2_gadget_) == 0) {
            return Slot.Middle;
        } else if (getValue(slot_1_gadget_) == 1 && getValue(slot_2_gadget_) == 0) {
            return Slot.Left;
        } else {
            return Slot.Right;
        }
    }
    
    private RobotOperation.Location getHeight() {
        if (getValue(height_1_gadget_) == 0 && getValue(height_2_gadget_) == 0) {
            return RobotOperation.Location.Middle;
        } else if (getValue(height_1_gadget_) == 1 && getValue(height_2_gadget_) == 0) {
            return RobotOperation.Location.Top;
        } else {
            return RobotOperation.Location.Bottom;
        }
    }

    @Override
    public void generateActions() {
        super.generateActions();

        SwimmyRobot2023 robot = ((SwimmyRobot2023)getSubsystem().getRobot());
        Swimmy2023RobotSubsystem robotSubsystem = ((Swimmy2023RobotSubsystem)robot.getRobotSubsystem());

        state_output1_.setState(State.OFF);
        state_output2_.setState(State.OFF);

        if (getValue(turtle_gadget_) == 1) {
            robotSubsystem.setAction(turtleAction);
            
        }
        else if (getValue(abort_gadget_) == 1) {
            robotSubsystem.abort();
        }
        else {
            RobotOperation operation = new RobotOperation();

            operation.setAction(getValue(collect_v_place_gadget_) == 1 ? RobotOperation.Action.Place : RobotOperation.Action.Collect);
            operation.setGamePiece(getGamePiece());
            operation.setAprilTag(getTag());
            operation.setManual(getValue(auto_v_manual_gadget_) == 1);
            operation.setSlot(getSlot());
            operation.setLocation(getHeight());

            if (getValue(action_gadget_) == 1) {
                operation.setGround(true);
                robotSubsystem.setOperation(operation);
            }
            else if (getValue(lock_gadget_) == 1) {
                robotSubsystem.setOperation(operation);
            }
        }
    }

    @Override
    protected void initializeLEDs() throws BadParameterTypeException, MissingParameterException {
        super.initializeLEDs();
        int num ; 

        num = getSubsystem().getSettingsValue("panel:outputs:state1").getInteger();
        state_output1_ = createLED(num);

        num = getSubsystem().getSettingsValue("panel:outputs:state2").getInteger();
        state_output2_ = createLED(num);
    }

    @Override
    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        super.initializeGadgets();
        int num;

        num = getSubsystem().getSettingsValue("panel:gadgets:collect_v_place").getInteger();
        collect_v_place_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:auto_v_manual").getInteger();
        auto_v_manual_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:lock").getInteger();
        lock_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger();
        abort_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:action").getInteger();
        action_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:drop").getInteger();
        drop_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:turtle").getInteger();
        turtle_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:cone_v_cube:1").getInteger();
        cone_v_cube_1_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:cone_v_cube:2").getInteger();
        cone_v_cube_2_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:apriltag:1").getInteger();
        apriltag_1_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:apriltag:2").getInteger();
        apriltag_2_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:slot:1").getInteger();
        slot_1_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:slot:2").getInteger();
        slot_2_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:height:1").getInteger();
        height_1_gadget_ = mapButton(num, ButtonType.Level);
        
        num = getSubsystem().getSettingsValue("panel:gadgets:height:2").getInteger();
        height_2_gadget_ = mapButton(num, ButtonType.Level);
    }
}
