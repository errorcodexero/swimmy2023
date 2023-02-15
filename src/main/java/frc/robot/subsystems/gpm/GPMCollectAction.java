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


        //
        // Over all, instead of doing these kinds of sequences with brute force. I would create an
        // ArmGotoAction that could read sets of positions from the settings file.  Then you could
        // do something like arm_collect_action = new ArmGotoAction(subsystem_.getArm(), "collect") or
        // arm_retract_action_ = new ArmGotoAction(subsystem_.getArm(), "retract").  Then this goto action
        // would execute the sequence of gotos.  This is useful in other places like the place actions.
        //
        // "arm" : {
        //   "collect" : {
        //     "1" : {
        //        "lower" : NUMBER,
        //        "upper" : NUMBER
        //     }
        //   },
        //   "retract" : {
        //     "1" : {
        //        "lower" : NUMBER,
        //        "upper" : NUMBER
        //     }
        //     "2" : {
        //        "lower" : NUMBER,
        //        "upper" : NUMBER
        //     }
        //     "3" : {
        //        "lower" : NUMBER,
        //        "upper" : NUMBER
        //     }
        //   }
        // }        
        //

        //
        // Butch: This is positioning the ARM for collecting.  Names matter, so I would
        //        name this arm_collect_action_ for instance.
        //
        arm_collect_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:extend");

        //
        // Butch: This is one of three positions used to retract the arm.  I would name this
        //        arm_retract1_action_
        //
        arm_retract_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:retract");

        //
        // Butch: Also create GrabberStartCollect() and GrabberStopCollect() actions that
        //        deal with both opening and closing the grabber as well as controlling the
        //        spinner motors.  We might also need a GrabberHoldAction() to hold a game piece.

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
