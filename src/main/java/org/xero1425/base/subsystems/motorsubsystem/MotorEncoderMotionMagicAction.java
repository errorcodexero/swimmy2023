package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MotorEncoderMotionMagicAction extends MotorAction {

    private double SetDoneDelay = 2.0 ;

    // The plot ID for plotting the motion
    int plot_id_ ;

    static int name_id_ = 0 ;

    // The columns to plot
    private String [] plot_columns_ = 
    { 
        "time (sec)", 
        "target (%%units%%)", "apos (%%units%%)",
        "avel (%%units%%)", "tpos (%%units%%)",
        "error (%%units%%)"
    } ;


    private enum State {
        Waiting,
        Running,
        Delaying,
        Complete
    }

    private static final double NearEndpoint = 3000 ;
    private static final double EndVelocity = 1500 ;

    public enum HoldType
    {
        None,
        AtCurrentPosition,
        AtTargetPosition
    } ;

    private double start_ ;
    private double target_ ;
    private HoldType hold_ ;
    private State state_ ;
    private double maxa_ ;
    private double maxv_ ;
    private int strength_ ;
    private double start_pos_ ;
    private XeroTimer delay_timer_ ;

    public MotorEncoderMotionMagicAction(MotorEncoderSubsystem sub, double target, double maxa, double maxv, int strength, HoldType holdtype) throws Exception {
        super(sub);

        target_ = target ;
        hold_ = holdtype ;
        maxa_ = maxa ;
        maxv_ = maxv ;
        strength_ = strength ;

        TalonFX talon = getSubsystem().getMotorController().getTalonFX() ;
        if (talon == null) {
            throw new BadMotorRequestException(getSubsystem().getMotorController(), "requested TalonFX motor controller on non TalonFX motor") ;
        }

        double kp = sub.getSettingsValue("magic:kp").getDouble() ;
        double ki = sub.getSettingsValue("magic:ki").getDouble() ;
        double kd = sub.getSettingsValue("magic:kd").getDouble() ;
        double kf = sub.getSettingsValue("magic:kf").getDouble() ;
        
        talon.config_kP(0, kp);
        talon.config_kI(0, ki);
        talon.config_kD(0, kd);
        talon.config_kF(0, kf);

        plot_id_ = sub.initPlot(sub.getName() + "-" + toString(plot_id_++)) ;
    }

    public double getDistance() {
        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        return Math.abs(me.getPosition() - start_pos_) ;
    }

    public boolean isWaiting() {
        return state_ == State.Waiting ;
    }

    public boolean isRunning() {
        return state_ == State.Running ;
    }

    public boolean isComplete() {
        return state_ == State.Complete ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        start_ = getSubsystem().getRobot().getTime() ;
        state_ = State.Waiting ;
        tryStart() ;
    }

    @Override
    public void run() throws Exception  {
        super.run() ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        State old = state_ ;

        tryStart();

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        TalonFX talon = getSubsystem().getMotorController().getTalonFX() ;
        double mcvel = talon.getSelectedSensorVelocity() ;
        double delta = Math.abs(target_ - me.getPosition()) ;

        if (state_ == State.Running) {
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("MotionMagic Pos") ;
            logger.add("target", target_, "%.0f");
            logger.add("mctarget", talon.getClosedLoopTarget(), "%.0f");
            logger.add("actual", me.getPosition(), "%.0f") ;
            logger.add("velocity", mcvel, "%.0f") ;
            logger.add("state", state_.toString()) ;
            logger.endMessage();
        }
        else if (state_ == State.Delaying && delay_timer_.isExpired())
        {
            setDone() ;
            me.endPlot(plot_id_);
        }
        else if (state_ == State.Running && delta < NearEndpoint && Math.abs(mcvel) < EndVelocity) {
            state_ = State.Complete ;
            logger.startMessage(MessageType.Info) ;
            logger.add("Motion magic duration ") ;
            logger.add(getSubsystem().getRobot().getTime() - start_) ;
            logger.add("delta", delta) ;
            logger.endMessage();

            if (SetDoneDelay > 0.0) 
            {
                state_ = State.Delaying ;
                delay_timer_ = new XeroTimer(getSubsystem().getRobot(), "delaytimer", 2.0);
                delay_timer_.start() ;
            }
            else 
            {
                me.endPlot(plot_id_);
                setDone() ;
            }
        }

        Double[] data = new Double[plot_columns_.length] ;
        data[0] = getSubsystem().getRobot().getTime() - start_ ;
        data[1] = target_ ;
        data[2] = me.getPosition() ;
        data[3] = talon.getSelectedSensorVelocity() ;
        data[4] = talon.getClosedLoopTarget() ;
        data[5] = talon.getClosedLoopError() ;
        me.addPlotData(plot_id_, data);

        if (state_ != old) {
            logger.startMessage(MessageType.Info) ;
            logger.add("Motion Magic: ").add(old.toString()).add(" -> ").add(state_.toString()).endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        String ret ;

        ret = spaces(indent) + "MotorEncoderMotionMagicAction (" + getSubsystem().getName() + ")";
        ret += " target=" + target_ + ", hold=" + hold_.toString();

        return ret ;
    }

    private void tryStart() throws BadMotorRequestException {
        if (state_ == State.Waiting) {
            MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
            start_pos_ = me.getPosition();
            me.startPlot(plot_id_, convertUnits(plot_columns_, me.getUnits()));
            TalonFX talon = me.getMotorController().getTalonFX() ;
            talon.configMotionAcceleration(maxa_);
            talon.configMotionCruiseVelocity(maxv_);
            talon.configMotionSCurveStrength(strength_);
            talon.set(TalonFXControlMode.MotionMagic, target_);
            state_ = State.Running ;

            MessageLogger logger = me.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("MotionMagic: ").add("accel", maxa_, "%.0f").add("velocity", maxv_, "%.0f")
                    .add("strength", strength_).add("target", target_, "%.0f").endMessage() ;
        }
    }
}
