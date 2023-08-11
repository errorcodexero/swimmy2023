package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.TalonFXMotorController;
import org.xero1425.base.motors.MotorController.PidType;
import org.xero1425.base.plotting.PlotDataSource;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This action causes a MotorEncoderSubsystem to maintain a constant velocity.
public class MotorEncoderVelocityAction extends MotorAction {   

    // An index used to ensure individual instances of this action produce separate plots
    private static int which_ = 1 ;

    // The target velocity
    private double target_ ;

    // The error in the last robot loop
    private double error_ ;

    // The start time for the action
    private double start_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // Plot data source
    private PlotDataSource plot_src_ ;

    // The name of the action
    private String name_ ;

    // The duration of the plot request
    private double plot_duration_ ;

    // The timer for the plot
    private XeroTimer plot_timer_ ;

    /// \brief Create a new MotorEncoderVelocityAction
    /// \param sub the target MotorEncoderSubsystem
    /// \param name the name of the action, for entries from the settings file
    /// \param target the traget velocity
    public MotorEncoderVelocityAction(MotorEncoderSubsystem sub, String name, double target)
            throws MissingParameterException, BadParameterTypeException, BadMotorRequestException, MotorRequestFailedException {

        super(sub);

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;

        name_ = name ;
        target_ = target;

        String pidname = "subsystems:" + sub.getName() + ":" + name_ ;

        plot_duration_ = 10.0 ;
        if (settings.isDefined(pidname + ":plot-duration")) {
            plot_duration_ = settings.get(pidname + ":plot-duration").getDouble() ;
        }
        plot_timer_ = new XeroTimer(sub.getRobot(), "velocity-action-plot", plot_duration_) ;
        createPlotDataSource();
    }

    /// \brief Create a new MotorEncoderVelocityAction
    /// \param sub the target MotorEncoderSubsystem
    /// \param target a string with the name of the target velocity in settings file
    public MotorEncoderVelocityAction(MotorEncoderSubsystem sub, String name, String target) throws BadParameterTypeException, MissingParameterException, BadMotorRequestException, MotorRequestFailedException {
        super(sub) ;

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
        String pidname = "subsystems:" + sub.getName() + ":" + name_ ;

        target_ = getSubsystem().getSettingsValue(target).getDouble() ;

        plot_duration_ = 10.0 ;
        if (settings.isDefined(pidname + ":plot-duration")) {
            plot_duration_ = settings.get(pidname + ":plot-duration").getDouble() ;
        }
        plot_timer_ = new XeroTimer(sub.getRobot(), "velocity-action-plot", plot_duration_) ;        

        createPlotDataSource();
    }

    private void createPlotDataSource() {
        plot_id_ = getSubsystem().initPlot(toString(0) + "-" + String.valueOf(which_++)) ; 

        plot_src_ = new PlotDataSource() ;

        plot_src_.addDataElement("time", () -> { return getSubsystem().getRobot().getTime() - start_ ;});
        plot_src_.addDataElement("target (%%units%%)", () -> { return target_ ; });
        plot_src_.addDataElement("actual (%%units%%)", () -> { return ((MotorEncoderSubsystem)(getSubsystem())).getPosition() ; });
        plot_src_.addDataElement("error (%%units%%)", () -> { return error_ ; });
    }

    public double getError() {
        return error_ ;
    }

    /// \brief Return the name of the action
    /// \returns the name of the action
    public String getName() {
        return name_ ;
    }

    /// \brief Update the target velocity to a new velocity
    /// \param target the target velocity desired
    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        target_ = target ;

        //
        // If we are running the loop in the motor controller, commuincate the new target to the
        // motor controller.  Since the motor controller does not run its PID loop in robot units,
        // we must as the subsystem to translate the units from robot units to motor controller units.
        //
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        MotorController mc = sub.getMotorController() ;
        TalonFXMotorController talon = (TalonFXMotorController)mc ;
        if (talon != null) {
            //
            // If this is a Talon FX, we need to supply velocities in encoder units
            // per 100 ms.  If this is a SparkMax we program the SparkMax to work in native
            // units.
            //
            target = sub.getEncoder().mapVelocityToMotor(target) / 10.0 ;
        }
        getSubsystem().getMotorController().setTarget(target);
    }

    /// \brief Returns the current target
    /// \returns the current target
    public double getTarget() {
        return target_ ;
    }

    /// \brief Start the velocity action
    @Override
    public void start() throws Exception {
        super.start() ;

        if (plot_id_ != -1) {
            getSubsystem().startPlot(plot_id_, plot_src_) ;
            plot_timer_.start() ;
        }

        start_ = getSubsystem().getRobot().getTime() ;

        //
        // We are using a control loop in the motor controller, get the parameters from the
        // settings file
        //
        double p = getSubsystem().getSettingsValue(name_ + ":kp").getDouble() ;
        double i = getSubsystem().getSettingsValue(name_ + ":ki").getDouble() ;
        double d = getSubsystem().getSettingsValue(name_ + ":kd").getDouble() ;
        double f = getSubsystem().getSettingsValue(name_ + ":kf").getDouble() ;
        double outmax = getSubsystem().getSettingsValue(name_ + ":max").getDouble() ;

        getSubsystem().getMotorController().setPID(PidType.Velocity, p, i, d, f, outmax);
        setTarget(target_);
    }

    /// \brief Process the velocity action once per robot loop, adjusting the power as needed
    @Override
    public void run() throws Exception {
        super.run() ;
        if (plot_id_ != -1) {
            getSubsystem().addPlotData(plot_id_);
            if (plot_timer_.isExpired()) {
                getSubsystem().endPlot(plot_id_) ;
                plot_id_ = -1 ;
            }
        }
    }

    /// \brief Cancel the velocity action, settings the power of the motor to zero
    @Override
    public void cancel() {
        super.cancel() ;

        try {
            getSubsystem().getMotorController().stopPID() ;
        }
        catch(Exception ex) {
        }

        getSubsystem().setPower(0.0);
        if (plot_id_ != -1)
            getSubsystem().endPlot(plot_id_) ;
    }

    /// \brief return a human readable string for the action
    /// \param indent the amount of white space prior to the description
    /// \returns a human readable string for the action
    @Override
    public String toString(int indent) {
        String ret = null ;

        ret = prefix(indent) + "MotorEncoderVelocityAction, " + getSubsystem().getName() + ", " +  Double.toString(target_) ;
        return ret ;
    }
}
