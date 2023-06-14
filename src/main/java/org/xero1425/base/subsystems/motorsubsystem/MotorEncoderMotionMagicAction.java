package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MotorEncoderMotionMagicAction extends MotorAction {

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
        Complete
    }

    private static final double NearEndpoint = 5000 ;
    private static final double EndVelocity = 3000 ;

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

        if (state_ == State.Running) {
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("MotionMagic Pos") ;
            logger.add("target", target_);
            logger.add("actual", me.getPosition()) ;
            logger.add("velocity", mcvel) ;
            logger.add("state", state_.toString()) ;
            logger.endMessage();
        }

        double delta = Math.abs(target_ - me.getPosition()) ;
        if (state_ == State.Running && delta < NearEndpoint && Math.abs(mcvel) < EndVelocity) {
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
            logger.add("delta", delta) ;
            logger.endMessage();
            me.endPlot(plot_id_);
            setDone() ;
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
            logger.add("MotionMagic: ").add("accel", maxa_).add("velocity", maxv_)
                    .add("strength", strength_).add("target", target_).endMessage() ;
        }
    }
}
