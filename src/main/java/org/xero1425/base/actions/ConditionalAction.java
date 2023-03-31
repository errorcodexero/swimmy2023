package org.xero1425.base.actions;

import java.util.function.Supplier;

import org.xero1425.misc.MessageLogger;

public class ConditionalAction extends Action {
    private Supplier<Boolean> condition_ ;
    private Action child_ ;
    private boolean running_ ;

    public ConditionalAction(MessageLogger logger, Supplier<Boolean> condition, Action child) {
        super(logger);

        condition_ = condition ;
        child_ = child ;
        running_ = false ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (condition_.get()) {
            child_.start() ;
            running_ = true ;
        }
        else {
            setDone() ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (child_.isDone()) {
            setDone();
        }
    }

    @Override
    public void cancel() {
        if (running_ && !child_.isDone()) {
            child_.cancel();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConditionalAction [" + child_.toString(0) + "]" ;
    }
}
