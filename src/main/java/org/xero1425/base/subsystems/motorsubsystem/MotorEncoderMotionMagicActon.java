package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MotorEncoderMotionMagicActon extends MotorAction {

    // The plot ID for plotting the motion
    int plot_id_ ;

    static int name_id_ = 0 ;

    // The columns to plot
    private String [] plot_columns_ = 
    { 
        "time (sec)", 
        "tpos (%%units%%)", "apos (%%units%%)", 
    } ;


    private enum State {
        Waiting,
        Running,
        Complete
    }

    private static final double NearEndpoint = 1000 ;

    public enum HoldType
    {
        None,
        AtCurrentPosition,
        AtTargetPosition
    } ;

    private double start_ ;
    private double target_ ;
    private double delay_ ;
    private HoldType hold_ ;
    private State state_ ;
    private double maxa_ ;
    private double maxv_ ;
    private int strength_ ;

    public MotorEncoderMotionMagicActon(MotorEncoderSubsystem sub, double delay, double target, double maxa, double maxv, int strength, HoldType holdtype) throws Exception {
        super(sub);

        target_ = target ;
        hold_ = holdtype ;
        maxa_ = maxa ;
        maxv_ = maxv ;
        strength_ = strength ;
        delay_ = delay ;

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
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem();
        getSubsystem().startPlot(plot_id_, convertUnits(plot_columns_, sub.getUnits()));

        start_ = getSubsystem().getRobot().getTime() ;
        state_ = State.Waiting ;
        tryStart() ;
    }

    @Override
    public void run() throws Exception  {
        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        State old = state_ ;

        tryStart();

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        if (state_ == State.Running) {
            logger.startMessage(MessageType.Info) ;
            logger.add("MotionMagic Pos").add("target", target_).add("actual", me.getPosition()) ;
            logger.endMessage();
        }
        if (state_ == State.Running && Math.abs(target_ - me.getPosition()) < NearEndpoint) {
            state_ = State.Complete ;
            switch(hold_) {
                case None:
                    break ;
                case AtCurrentPosition:
                    {
                        MotorEncoderHoldAction act = new MotorEncoderHoldAction(me);
                        getSubsystem().setDefaultAction(act) ;
                    }
                    break ;
                case AtTargetPosition:
                    {
                        MotorEncoderHoldAction act = new MotorEncoderHoldAction(me, target_);
                        getSubsystem().setDefaultAction(act) ;
                    }
                    break ;
            }
            logger.startMessage(MessageType.Info) ;
            logger.add("Motion magic duration ") ;
            logger.add(getSubsystem().getRobot().getTime() - start_) ;
            logger.endMessage();
            me.endPlot(plot_id_);
            setDone() ;
        }

        Double[] data = new Double[plot_columns_.length] ;
        data[0] = getSubsystem().getRobot().getTime() - start_ ;
        data[1] = target_ ;
        data[2] = me.getPosition() ;
        me.addPlotData(plot_id_, data);

        if (state_ != old) {

            logger.startMessage(MessageType.Info) ;
            logger.add("Motion Magic: ").add(old.toString()).add(" -> ").add(state_.toString()).endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "MotorEncoderMotionMagicActon delay=" + delay_ + ", target=" + target_ + ", hold=" + hold_.toString();
    }

    private void tryStart() throws BadMotorRequestException {
        if (state_ == State.Waiting) {
            double elapsed = getSubsystem().getRobot().getTime() - start_ ;
            if (elapsed > delay_) {
                TalonFX talon = getSubsystem().getMotorController().getTalonFX() ;
                talon.configMotionAcceleration(maxa_);
                talon.configMotionCruiseVelocity(maxv_);
                talon.configMotionSCurveStrength(strength_);
                talon.set(TalonFXControlMode.MotionMagic, target_);
                state_ = State.Running ;

                MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
                logger.startMessage(MessageType.Info) ;
                logger.add("MotionMagic: ").add("accel", maxa_).add("velocity", maxv_)
                        .add("strength", strength_).add("target", target_).endMessage() ;
            }
        }
    }
}
