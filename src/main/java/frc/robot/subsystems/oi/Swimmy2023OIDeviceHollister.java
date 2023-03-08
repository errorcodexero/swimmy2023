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
import frc.robot.subsystems.toplevel.OperationCtrl;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
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

    private DisplayPattern current_pattern_ ;
    private int collect_v_place_gadget_;
    private int auto_v_manual_gadget_;
    private int lock_gadget_;
    private int abort_gadget_;
    private int turtle_gadget_;

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

    private int station_ground_gadget_ ;

    private GPMStowAction turtle_action_ ;

    public Swimmy2023OIDeviceHollister(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);
    }

    @Override
    public void computeState() throws Exception {
        super.computeState();
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
    
    private RobotOperation.Location getHeight() {
        if (getValue(height_1_gadget_) == 0 && getValue(height_2_gadget_) == 0) {
            return RobotOperation.Location.Middle;
        } else if (getValue(height_1_gadget_) == 1 && getValue(height_2_gadget_) == 0) {
            return RobotOperation.Location.Top;
        } else {
            return RobotOperation.Location.Bottom;
        }
    }

    private boolean driverLock(RobotOperation oper) {
        Gamepad gp = getSubsystem().getGamePad();
        return oper.getGround() && gp.isRTriggerPressed();
    }

    private boolean isAbort() {
        if (getValue(abort_gadget_) == 1)
            return true ;

        Gamepad gp = getSubsystem().getGamePad();
        if (gp.isXPressed() && gp.isAPressed())
            return true ;

        return false ;
    }

    @Override
    public void generateActions() {
        super.generateActions();

        SwimmyRobot2023 robot = ((SwimmyRobot2023)getSubsystem().getRobot());
        Swimmy2023RobotSubsystem robotSubsystem = ((Swimmy2023RobotSubsystem)robot.getRobotSubsystem());

        if (robotSubsystem.isOperationComplete() == true && current_pattern_ != DisplayPattern.ERROR) {
            setDisplay(DisplayPattern.NONE);
        }

        if (getValue(turtle_gadget_) == 1) {
            robotSubsystem.getGPM().setAction(turtle_action_);
            setDisplay(DisplayPattern.NONE);
        }
        else if (isAbort()) {
            robotSubsystem.abort();
            setDisplay(DisplayPattern.NONE);
        } else {
            RobotOperation operation = new RobotOperation();

            operation.setAction(getValue(collect_v_place_gadget_) == 1 ? RobotOperation.Action.Place : RobotOperation.Action.Collect);
            operation.setGamePiece(getGamePiece());
            operation.setAprilTag(getTag());
            operation.setManual(getValue(auto_v_manual_gadget_) == 0);
            operation.setSlot(getSlot());
            operation.setLocation(getHeight());
            operation.setGround(getValue(station_ground_gadget_) == 1) ;
            
            if (getValue(lock_gadget_) == 1 || driverLock(operation)) {
                if (robotSubsystem.setOperation(operation)) {
                    if (operation.getGamePiece() == GamePiece.Cone) {
                        setDisplay(DisplayPattern.CONE);
                    }
                    else {
                        setDisplay(DisplayPattern.CUBE);                        
                    }
                } else {
                    setDisplay(DisplayPattern.ERROR);
                }
            }
            else {
                OperationCtrl running = robotSubsystem.getRunningController();

                if (running != null && running.getOper().getGround()) {
                    //
                    // We are actively running a ground operation
                    //
                    if (operation.getGamePiece() != running.getOper().getGamePiece() && operation.getGamePiece() != GamePiece.None) {
                        running.updateGamePiece(operation.getGamePiece());
                    }
                }
            }
        }
    }

    private void setDisplay(DisplayPattern pattern) {
        current_pattern_ = pattern ;
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

        num = getSubsystem().getSettingsValue("panel:gadgets:auto_v_manual").getInteger();
        auto_v_manual_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:lock").getInteger();
        lock_gadget_ = mapButton(num, ButtonType.LowToHigh);

        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger();
        abort_gadget_ = mapButton(num, ButtonType.LowToHigh);

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

        num = getSubsystem().getSettingsValue("panel:gadgets:station_v_ground").getInteger();
        station_ground_gadget_ = mapButton(num, ButtonType.Level);
    }
}
