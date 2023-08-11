package org.xero1425.base.subsystems.motorsubsystem ;

/// \file

/// \brief This class is a power action for the MotorEncoderSubsystem that applies a fixed power to the
/// motor and provides plots of the output in real world units
public class MotorEncoderMultiPowerAction extends MotorAction
{
    private double [] times_ ;

    private double [] powers_ ;

    private int current_index_ ;

    private double current_time_ ;

    private double start_ ;

    /// \brief Create the MotorEncoderPowerAction that applies a fixed power value then is done
    /// \param motor the subsystem to apply the action to
    /// \param power the power to apply to the motor
    public MotorEncoderMultiPowerAction(MotorEncoderSubsystem motor, double[] times, double[] powers) throws IllegalArgumentException {
        super(motor);

        if (times.length != powers.length) {
            throw new IllegalArgumentException("expected size of times array and powers array to be the same") ;
        }

        times_ = times ;
        powers_ = powers ;
    }

    /// \brief Start the action by applying the power requested
    @Override
    public void start() throws Exception {
        super.start() ;

        current_index_ = 0 ;
        start_ = getSubsystem().getRobot().getTime() ;
        current_index_ = 0 ;
        current_time_ = start_ ;

        getSubsystem().setPower(powers_[current_index_]) ;
    }

    /// \brief Called each robot loop.  Calls the base class to perform the action and then
    /// sends the plot data to the plot manager.
    @Override
    public void run() {

        double current_delta = getSubsystem().getRobot().getTime() - current_time_ ;
        if (current_delta > times_[current_index_]) {
            //
            // Move to the next item if there is one
            //
            current_index_++ ;
            if (current_index_ == powers_.length) {
                setDone() ;
            }
            else {
                current_time_ = getSubsystem().getRobot().getTime() ;
                getSubsystem().setPower(powers_[current_index_]) ;
            }
        }
    }

    /// \brief Cancel the action, settings the power to zero
    @Override
    public void cancel() {
        super.cancel() ;
    }

    /// \brief Returnsa human readable string describing the action
    /// \returns a human readable string describing the action
    @Override
    public String toString(int indent) {
        String ret ;

        ret = spaces(indent) + toString() ;
        return ret ;
    }

    @Override
    public String toString() {
        return "MotorEncoderMultiPowerAction " + getSubsystem().getName() ;
    }
}
