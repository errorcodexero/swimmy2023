package frc.robot.automodes;

import org.xero1425.base.actions.Action;
import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;

public class SwimmyPitTestAutomode extends AutoMode {
    public SwimmyPitTestAutomode(SwimmyRobotAutoController ctrl) throws Exception {
        super(ctrl, "PitTest");

        Action act ;
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();
        GrabberSubsystem grabber = robot.getGPM().getGrabber();
        ArmSubsystem arm = robot.getGPM().getArm() ;

        act = new MotorEncoderGotoAction(grabber.getGrabSubsystem(), 750, true);
        addSubActionPair(grabber.getGrabSubsystem(), act, true);

        addAction(new DelayAction(ctrl.getRobot(), 1.0));

        act = new MotorEncoderGotoAction(grabber.getGrabSubsystem(), 25, true);
        addSubActionPair(grabber.getGrabSubsystem(), act, true);
        
        act = new MotorEncoderPowerAction(grabber.getSpinSubsystem(), 0.4, 2.0) ;
        addSubActionPair(grabber.getSpinSubsystem(), act, true);

        act = new MotorEncoderPowerAction(grabber.getSpinSubsystem(), -0.4, 2.0) ;
        addSubActionPair(grabber.getSpinSubsystem(), act, true);

        act = new MotorEncoderGotoAction(arm.getUpperSubsystem(), 20000, true);
        addSubActionPair(arm.getUpperSubsystem(), act, true);

        act = new MotorEncoderGotoAction(arm.getLowerSubsystem(), 60000, true);
        addSubActionPair(arm.getLowerSubsystem(), act, true);

        addAction(new DelayAction(ctrl.getRobot(), 1.0));

        act = new MotorEncoderGotoAction(arm.getLowerSubsystem(), 2000, true);
        addSubActionPair(arm.getLowerSubsystem(), act, true);

        act = new MotorEncoderGotoAction(arm.getLowerSubsystem(), 0, true);
        addSubActionPair(arm.getLowerSubsystem(), act, true);
    }
}
