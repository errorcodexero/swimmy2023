package frc.robot.subsystems;

import org.xero1425.base.subsystems.Subsystem;

public class GPMSubsystem extends Subsystem {
    private ArmSubsystem armSubsystem_;
    private GrabberSubsystem grabberSubsystem_;

    public GPMSubsystem(Subsystem parent) throws Exception {
        super(parent, "gpm");
        
        armSubsystem_ = new ArmSubsystem(this);
        addChild(armSubsystem_);
        
        grabberSubsystem_ = new GrabberSubsystem(this);
        addChild(grabberSubsystem_);
    }

    public ArmSubsystem getArm() {
        return armSubsystem_ ;
    }

    public GrabberSubsystem getGrabber() {
        return grabberSubsystem_ ;
    }
}
