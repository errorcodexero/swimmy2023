package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.plotting.PlotDataSource;
import org.xero1425.misc.PIDCtrl;

public class MotorEncoderTrackPositionAction extends MotorAction {
    
    // The target position
    private double target_ ;

    // The PID controller to follow the plan
    private PIDCtrl ctrl_ ;

    // The error between the subsystem position and the target
    private double error_ ;

    // The time the last loop was run
    private double last_time_ ;

    // The start time for the action
    private double start_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // Data source for plots
    private PlotDataSource plot_src_ ;

    /// \brief Create the action
    /// \param sub the MotorEncoderSubsystem subsystem for the action    
    /// \param target the target position
    /// \param addhold if true, add a hold action when the goto action is complete
    public MotorEncoderTrackPositionAction(MotorEncoderSubsystem sub, String name, double target) throws Exception {
        super(sub) ;

        if (!(sub instanceof MotorEncoderSubsystem))
            throw new Exception("This subsystem is not a MotorEncoderSubsystem") ;
                    
        target_ = checkTarget(target) ;
        
        ctrl_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), "subsystems:" + sub.getName() + ":" + name, false) ;
        createPlotDataSource() ;
    }

    private void createPlotDataSource() {
        plot_src_ = new PlotDataSource() ;

        plot_src_.addDataElement("time", () -> { return getSubsystem().getRobot().getTime() - start_ ;});
        plot_src_.addDataElement("target (%%units%%)", () -> { return target_ ; });
        plot_src_.addDataElement("actual (%%units%%)", () -> { return ((MotorEncoderSubsystem)(getSubsystem())).getPosition() ; });
        plot_src_.addDataElement("error (%%units%%)", () -> { return error_ ; });
        plot_src_.addDataElement("out (volts)", () -> { return getSubsystem().getPower() ; });
        plot_id_ = getSubsystem().initPlot(toString(0), plot_src_) ;
    }

    public double getError() {
        return error_ ;
    }

    /// \brief Start the action, computing the plan using the trapezoidal profile.  This method also
    /// initializes the PID controller based on whether or not the motion us "up" or "down".
    public void start() throws Exception {
        super.start() ;

        start_ = getSubsystem().getRobot().getTime() ;

        if (plot_id_ != -1) {
            MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem();
            plot_src_.convertUnits(sub.getUnits());
            getSubsystem().startPlot(plot_id_);
        }
    }

    public void setTarget(double t) {
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;

        target_ = checkTarget(t) ;
        error_ = Math.abs(target_ - sub.getPosition()) ;
    }

    private double checkTarget(double t) {
        // MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;

        // if (t < sub.getMinPos())
        //     t = sub.getMinPos() ;

        // if (t > sub.getMaxPos())
        //     t = sub.getMaxPos() ;

        return t ;
    }

    public double getTarget() {
        return target_ ;
    }

    /// \brief Called once per robot loop to adjust the motor power to follow the motion plan
    // given by the TrapezoidalProfile class.
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        double t = sub.getRobot().getTime() ;

        double out = ctrl_.getOutput(target_, sub.getPosition(), t - last_time_) ;
        sub.setPower(out) ;

        last_time_ = t ;

        error_ = Math.abs(target_ - sub.getPosition()) ;

        if (plot_id_ != -1) {
            if (getSubsystem().getRobot().getTime() - start_ > 2.5)
            {
                getSubsystem().endPlot(plot_id_) ;
                plot_id_ = -1 ;
            }
        }        
    }

    /// \brief Cancel the action and set the motor power to zero
    public void cancel() {
        super.cancel() ;

        if (plot_id_ == -1) {
            getSubsystem().endPlot(plot_id_) ;
            plot_id_ = -1 ;            
        }

        getSubsystem().setPower(0.0) ;
    }

    /// \brief Returns a human readable string describing the action
    /// \returns a human readable string describing the action
    public String toString(int indent) {
        return prefix(indent) + "MotorEncoderTrackPositionAction," + getSubsystem().getName() + "," + Double.toString(target_) ;
    }
}
