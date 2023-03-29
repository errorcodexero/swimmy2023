package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OILed;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OILed.State;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.SwimmyRobot2023;
import frc.robot.subsystems.gpm.GPMStowAction;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

//
// Switches, purpose, and types
//
// Collect vs Place     Single Toggle (1 DIO)
// Lock                 Button (1 DIO)
// Abort                Button (1 DIO)
// Turtle               Button (1 DIO)
// Collect              Button (1 DIO)
// Cone vs Cube         Single Toggle (1 DIO)
// April Tag            Three way toggle (2 DIOs)
// Slot                 Three way toggle (2 DIOs)
// Height               Three way toggle (2 DIOs)
//

public class Swimmy2023OIDeviceHollister extends OIPanel {
    private enum DisplayPattern {
        NONE(3), 
        CONE(1),
        CUBE(2),
        ERROR(0);

        /// The value of the enum
        public final int value ;

        private DisplayPattern(int value) {
            this.value = value ;
        }  
    } ;

    private int collect_v_place_gadget_;
    private int lock_gadget_;
    private int abort_gadget_;
    private int turtle_gadget_;
    private int collect_gadget_ ;

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

    private GPMStowAction turtle_action_ ;

    private DisplayPattern current_display_ ;

    public Swimmy2023OIDeviceHollister(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);

        current_display_ = DisplayPattern.NONE ;
    }

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }

    public boolean isCollectButtonPressed() {
        return getValue(collect_gadget_) == 1 ;
    }
       
    @Override
    public void createStaticActions() throws Exception {
        SwimmyRobot2023 robot = (SwimmyRobot2023)getSubsystem().getRobot();
        Swimmy2023RobotSubsystem subsystem = (Swimmy2023RobotSubsystem)robot.getRobotSubsystem();

        turtle_action_ = new GPMStowAction(subsystem.getGPM());
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
    
    private RobotOperation.Location getLocation() {
        if (getValue(height_1_gadget_) == 0 && getValue(height_2_gadget_) == 0) {
            return Location.Middle;
        } else if (getValue(height_1_gadget_) == 1 && getValue(height_2_gadget_) == 0) {
            return Location.Top;
        } else {
            return Location.Bottom;
        }
    }

    private RobotOperation.Action getAction() {
        RobotOperation.Action act = Action.Collect ;
        if (getValue(collect_v_place_gadget_) == 1) {
            act = Action.Place ;
        }
        return act ;
    }

    private boolean isDriverGroundCollect() {
        Gamepad gp = getSubsystem().getGamePad();
        return gp.isRTriggerPressed();
    }

    private boolean isDriverManualStationCollect() {
        Gamepad gp = getSubsystem().getGamePad();
        return gp.isRBackButtonPressed();
    }

    private boolean isAbort() {
        if (getValue(abort_gadget_) == 1)
            return true ;

        Gamepad gp = getSubsystem().getGamePad();
        if (gp != null && gp.isXPressed() && gp.isAPressed())
            return true ;

        return false ;
    }

    @Override
    public void generateActions() {
        super.generateActions();

        SwimmyRobot2023 robot = ((SwimmyRobot2023)getSubsystem().getRobot());
        Swimmy2023RobotSubsystem robotSubsystem = ((Swimmy2023RobotSubsystem)robot.getRobotSubsystem());
        RobotOperation operation = null ;
        GamePiece gp = getGamePiece() ;

        //
        // Robot always reflects the state of the cube/cone switch on the OI
        //
        robotSubsystem.setDisplayState(gp);

        if (robotSubsystem.getRunningController() == null && current_display_ != DisplayPattern.NONE) {
            //
            // If the robot subsystem has finished a controller, blank the OI display
            //
            setDisplay(DisplayPattern.NONE);
        }

        if (getValue(turtle_gadget_) == 1) {
            robotSubsystem.getGPM().setAction(turtle_action_);
            setDisplay(DisplayPattern.NONE);
        }
        else if (isAbort()) {
            robotSubsystem.abort();
            robotSubsystem.cancelAction();
            setDisplay(DisplayPattern.NONE);
        } else if (isDriverGroundCollect()) {
            operation = new RobotOperation(gp, true);
        }
        else if (isDriverManualStationCollect()) {
            operation = new RobotOperation(gp, false);
        }
        else if (getValue(lock_gadget_) == 1) {
            operation = new RobotOperation();
            operation.setAction(getAction());
            operation.setGamePiece(gp);
            operation.setAprilTag(getTag());
            operation.setSlot(getSlot());
            operation.setLocation(getLocation());
            operation.setGround(false);
            operation.setManual(false);
        }

        if (operation != null) {
            if (!robotSubsystem.setOperation(operation)) {
                setDisplay(DisplayPattern.ERROR);
            }
            else if (gp == GamePiece.Cone) {
                setDisplay(DisplayPattern.CONE);
            }
            else if (gp == GamePiece.Cube) {
                setDisplay(DisplayPattern.CUBE);
            }
        }
    }

    private void setDisplay(DisplayPattern pattern) {
        current_display_ = pattern ;

        int value = pattern.value ;
        if ((value & 1) == 0) {
            state_output1_.setState(State.OFF);
        }
        else {
            state_output1_.setState(State.ON);
        }

        if ((value & 2) == 0) {
            state_output2_.setState(State.OFF);
        }
        else {
            state_output2_.setState(State.ON);
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

        num = getSubsystem().getSettingsValue("panel:gadgets:lock").getInteger();
        lock_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger();
        abort_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:turtle").getInteger();
        turtle_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:collect").getInteger();
        collect_gadget_ = mapButton(num, ButtonType.LowToHigh);

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
