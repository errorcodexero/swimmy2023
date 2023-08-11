package org.xero1425.base.subsystems.tankdrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDACtrl;
import org.xero1425.misc.TrapezoidalProfile;
import org.xero1425.misc.XeroMath;

public class TankDriveRotateAction extends TankDriveAction {

    private double target_ ;
    private double threshold_ ;
    private double start_position_ ;
    private double start_time_ ;

    // The PID controller to follow the plan
    PIDACtrl ctrl_ ;

    // The TrapezoidalProfile that is the plan to follow
    TrapezoidalProfile profile_ ;

    // ID number to add to each action
    static int name_id_ = 0 ;
    
    public TankDriveRotateAction(TankDriveSubsystem sub, double target) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        target_ = target ;

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
        profile_ = new TrapezoidalProfile(settings, "subsystems:" + sub.getName() + ":rotate:trapezoid") ;
    }    

    @Override
    public void start() throws Exception {
        super.start() ;

        setTarget();
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        TankDriveSubsystem sub = (TankDriveSubsystem)getSubsystem() ;
        XeroRobot robot = sub.getRobot() ;

        double dt = robot.getDeltaTime() ;
        double elapsed = robot.getTime() - start_time_ ;
        double position = sub.getAngle().getDegrees() ;
        double traveled = XeroMath.normalizeAngleDegrees(position - start_position_) ;

        if (elapsed > profile_.getTotalTime())
        {
            setDone() ;
            sub.setPower(0.0, 0.0) ;
        }
        else
        {
            double targetDist = profile_.getDistance(elapsed) ;
            double targetVel = profile_.getVelocity(elapsed) ;
            double targetAcc = profile_.getAccel(elapsed) ;
            double out = ctrl_.getOutput(targetAcc, targetVel, targetDist, traveled, dt) ;
            sub.setPower(out, -out) ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        TankDriveSubsystem sub = (TankDriveSubsystem)getSubsystem() ;
        sub.setPower(0.0, 0.0);
        setDone() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "TankDriveRotateAction " + target_ ;
    }

    private void setTarget() throws BadParameterTypeException, MissingParameterException {
        TankDriveSubsystem sub = (TankDriveSubsystem)getSubsystem() ;

        // Check the current position to see if we are done
        double dist = XeroMath.normalizeAngleDegrees(target_ = sub.getAngle().getDegrees()) ;
        if (Math.abs(dist) < threshold_)
        {
            setDone() ;
        }
        else
        {
            //
            // Initialize the follower
            //
            String config = "subsystems:" + sub.getName() + ":rotate:follower" ;
            ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
            ctrl_ = new PIDACtrl(settings, config, true) ;

            // Update the trapezoidal profile based on when we are starting.
            profile_.update(dist, 0, 0) ;
            start_time_ = sub.getRobot().getTime() ;
            start_position_ = sub.getAngle().getDegrees() ;
            start_time_ = sub.getRobot().getTime() ;
        }
    }    
}
