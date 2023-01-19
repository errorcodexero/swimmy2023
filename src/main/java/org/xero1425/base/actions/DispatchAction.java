package org.xero1425.base.actions;

import java.util.List;

import org.xero1425.base.subsystems.Subsystem;

/// \file

/// \brief this action dispatachs a child action to a subsystem
public class DispatchAction extends ActionGroup {
    
    // The subsystem for the action
    private Subsystem sub_;

    // The action to assign to the subsystem
    private Action act_;

    // if true, this action blocks until the child action completes, otherwise
    // the child action is assigned and this action is complete.
    private boolean block_;

    /// \brief create a new dispatch action
    /// \param sub the sbusystem to receive the child action
    /// \param act the child action to assign
    /// \param block if true, this action blocks until the child action is complete
    public DispatchAction(Subsystem sub, Action act, boolean block) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        act_ = act;
        block_ = block;
    }

    /// \brief start the DispatchAction by assigning the child action to the target subsystem
    @Override
    public void start() throws Exception {
        super.start();

        sub_.setAction(act_);
        if (!block_)
            setDone();
    }

    /// \brief run the DispatchAction.
    /// This is only called when this DIspathAction is blocking.  In that case, this method
    /// monitors the child action until its done state is true, and then sets the done state of this
    /// action to true.
    @Override
    public void run() throws Exception {
        super.run();

        if (block_ && act_.isDone()) {
            setDone();
        }
    }

    /// \brief cancel this action
    /// This method has no effect if this action is not blocking.  If this action is blocking, this
    /// method cancels the child action and assigns a null action to the subsystem.
    @Override
    public void cancel() {
        super.cancel();
        if (block_) {
            act_.cancel();
            sub_.setAction(null);
        }
    }

    /// \brief returns a human readable string descrbing this DispatchAction
    /// \returns a human readable string descrbing this DispatchAction
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "DispatchAction, ";

        ret += sub_.getName() ;
        if (block_)
            ret += ", BLOCKING";
        else
            ret += ", NONBLOCKING";

        ret += "  [" ;
        ret += "\n" ;
        if (act_ == null)
            ret += "(null)" ;
        else
            ret += act_.toString(indent + 4) ;
        ret += "\n" ;
        ret += spaces(indent) + "]" ;
        return ret;
    }


    /// \brief places the child action into the output list
    @Override
    public void getAllChildren(List<Action> output) {
        if (act_ instanceof ActionGroup)
            ((ActionGroup)act_).getAllChildren(output);
            
        output.add(act_) ;
    }
}
