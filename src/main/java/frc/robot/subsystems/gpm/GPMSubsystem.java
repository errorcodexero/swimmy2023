package frc.robot.subsystems.gpm;

import org.xero1425.base.subsystems.Subsystem;

import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;

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
    
    protected void computeMyState() {
    }    

    public ArmSubsystem getArm() {
        return armSubsystem_ ;
    }

    public GrabberSubsystem getGrabber() {
        return grabberSubsystem_ ;
    }
}
