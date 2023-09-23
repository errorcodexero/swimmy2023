package org.xero1425.base.actions;

import java.util.function.Supplier;
import org.xero1425.misc.MessageLogger;

public class ConditionalAction extends Action {
    private Supplier<Boolean> condition_ ;
    private Action iftrue_ ;
    private Action iffalse_ ;
    private boolean state_ ;

    public ConditionalAction(MessageLogger logger, Supplier<Boolean> condition, Action iftrue, Action iffalse) {
        super(logger);

        state_ = false ;
        condition_ = condition ;
        iftrue_ = iftrue ;
        iffalse_ = iffalse ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = condition_.get();
        if (state_) {
            iftrue_.start() ;
        }
        else {
            if (iffalse_ != null) {
                iffalse_.start() ;
            }
            else {
                setDone() ;
            }
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (state_ && iftrue_.isDone()) {
            setDone() ;
        }
        else if (!state_ && iffalse_.isDone()) {
            setDone();
        }
    }

    @Override
    public void cancel() {
        if (state_) {
            iftrue_.cancel() ;
        }
        else {
            iffalse_.cancel() ;
        }
    }

    @Override
    public String toString(int indent) {
        String ret  = spaces(indent) + "ConditionalAction [" ;
        ret += iftrue_.toString(0) ;
        if (iffalse_ != null) {
            ret += ", " ;
            ret += iffalse_.toString(0) ;
        }
        ret += "]" ;
        return ret;
    }
}
