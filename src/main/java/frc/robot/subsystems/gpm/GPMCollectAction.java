package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberStartCollectAction;
import frc.robot.subsystems.grabber.GrabberStopCollectAction;

public class GPMCollectAction extends Action {

    GPMSubsystem subsystem_;
    GrabberStartCollectAction grabber_start_collect_action_;
    GrabberStopCollectAction grabber_stop_collect_action_;

    ArmGotoAction arm_collect_action_;
    ArmGotoAction arm_retract_action_;

    public GPMCollectAction(GPMSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());

        subsystem_ = subsystem;

        arm_collect_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:extend");
        arm_retract_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:retract");
        grabber_stop_collect_action_ = new GrabberStopCollectAction(subsystem.getGrabber());
        grabber_start_collect_action_ = new GrabberStartCollectAction(subsystem_.getGrabber());
    }

    @Override
    public void start() throws Exception {
        super.start();

        subsystem_.getGrabber().setAction(grabber_start_collect_action_, true);
        subsystem_.getArm().setAction(arm_collect_action_, true);
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (arm_collect_action_.isDone() && grabber_start_collect_action_.isDone()) {
            if (subsystem_.getGrabber().sensor()) {
                subsystem_.getGrabber().setAction(grabber_stop_collect_action_, true);
                subsystem_.getArm().setAction(arm_retract_action_, true);
                setDone() ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMCollectAction";
    }
}
