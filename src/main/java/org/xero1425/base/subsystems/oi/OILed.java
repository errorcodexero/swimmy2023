package org.xero1425.base.subsystems.oi;

public class OILed {
    public enum State {
        ON,
        OFF,
        BLINK_FAST,
        BLINK_SLOW
    }

    private OIPanel panel_ ;
    private int index_ ;
    private State state_ ;
    private double next_blink_ ;
    private boolean current_state_ ;
    private double blink_time_ ;

    private final double FastBlinkTime = 0.25 ;
    private final double SlowBlinkTime = 1.0 ;

    public OILed(OIPanel panel, int index) {
        panel_ = panel ;
        index_ = index ;
        state_ = State.OFF ;
    }

    public OIPanel getPanel() {
        return panel_ ;
    }

    public int getIndex() {
        return index_ ;
    }

    public void setState(State st) {

        if (state_ != st) {
            state_ = st ;

            if (state_ == State.BLINK_FAST)
                blink_time_ = FastBlinkTime ;
            else if (state_ == State.BLINK_SLOW)
                blink_time_ = SlowBlinkTime ;
                
            current_state_ = false ;
            next_blink_ = panel_.getSubsystem().getRobot().getTime()  + blink_time_ ;
        }
    }

    public State getState() {
        return state_ ;
    }

    protected void run() {
        if (state_ == State.OFF) {
            panel_.setOutput(index_, true) ;
        }
        else if (state_ == State.ON) {
            panel_.setOutput(index_, false) ;
        }
        else {
            double time = panel_.getSubsystem().getRobot().getTime() ;
            if (time > next_blink_) {
                next_blink_ = time + blink_time_ ;
                current_state_ = !current_state_ ;
                panel_.setOutput(index_, current_state_) ;
            }
        }
    }
}
