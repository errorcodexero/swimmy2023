package frc.robot.subsystems.oi;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.subsystems.oi.OIDevice;
import org.xero1425.base.subsystems.oi.OIPanel;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.OIPanelButton.ButtonType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.RobotOperation;

public class Swimmy2023OIDevice_V2 extends OIPanel {
    private int collect_v_place_gadget_;
    private int auto_v_manual_gadget_;
    private int lock_gadget_;
    private int abort_gadget_;
    private int turtle_gadget_;
    private int action_gadget_;
    private int cone_v_cube_gadget_;

    public Swimmy2023OIDevice_V2(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);
    }

    @Override
    public void computeState() throws Exception {
        super.computeState();
    }

    public void generateStaticActions() {
    }

    @Override
    public void generateActions() throws InvalidActionRequest {
        super.generateActions();
        
        if (getValue(lock_gadget_) == 1) {
            var operation = new RobotOperation();

            operation.setAction(getValue(collect_v_place_gadget_) == 1 ? RobotOperation.Action.PLACE
                    : RobotOperation.Action.COLLECT);
            operation.setAuto(getValue(auto_v_manual_gadget_) == 1);
        }
        if (getValue(turtle_gadget_) == 1) {
            // do turtle
        }
        if (getValue(abort_gadget_) == 1) {
            // do abort
        }
        if (getValue(action_gadget_) == 1) {
            // do action
        }

    }

    protected void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        int num;

        num = getSubsystem().getSettingsValue("panel:gadgets:collect_v_place").getInteger();
        collect_v_place_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:auto_v_manual").getInteger();
        auto_v_manual_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:lock").getInteger();
        lock_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:abort").getInteger();
        abort_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:action").getInteger();
        action_gadget_ = mapButton(num, ButtonType.Level);

        num = getSubsystem().getSettingsValue("panel:gadgets:turtle").getInteger();
        turtle_gadget_ = mapButton(num, ButtonType.Level);

    }

}
