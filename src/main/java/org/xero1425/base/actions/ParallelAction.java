package org.xero1425.base.actions;

import java.util.List;
import java.util.ArrayList;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MessageLogger;

/// \file

/// \brief This action executes a set of child actions in parallel.
/// This action is done only when all of the actions
public class ParallelAction extends ActionGroup
{
    // The list of child actions to execute
    private List<Action> actions_ ;

    // If true, this action is running
    private boolean running_ ;

    // The done behavior for this action
    private DonePolicy done_policy_ ;

    /// \brief This value indicates the policy for the parallel action being done
    public enum DonePolicy
    {
        All,            ///< The parallel action is done when all of its children are done
        First,          ///< THe parallel action is done when one of its children are done
    }

    /// \brief create a parallel action
    /// \param logger the message logger for the robot
    /// \param done the done policy for this parallel method
    public ParallelAction(MessageLogger logger, DonePolicy done) {
        super(logger) ;

        actions_ = new ArrayList<Action>() ;
        running_ = false ;
        done_policy_ = done ;
    }

    /// \brief get the done policy for this parallel action
    /// \returns the done policy for this parallel action
    DonePolicy getDonePolicy() {
        return done_policy_ ;
    }

    /// \brief add a child action to the parallel action
    /// \param act the action to add to the parallel action.
    public void addAction(Action act) throws InvalidActionRequest {
        if (running_)
            throw new InvalidActionRequest(this, InvalidActionRequest.Reason.ModifyingRunningAction,
                    "cannot add actions to parallel after it has been started") ;

        actions_.add(act) ;
    }

    /// \brief add a child action to be assigned to a subsystem to this parallel action
    /// Internally this method uses the DispatchAction.  The DisplayAction is the action that
    /// is actually added to the ParallelAction.  The action given by the act argument is a child
    /// action of the DispatchAction.
    /// \param sub the subsytem to receive the action
    /// \param act the action to assign to the subsystem
    /// \param block if true, this child action is not completed until the action assigned to the subsystem is complete
    public void addSubActionPair(Subsystem sub, Action act, boolean block) throws InvalidActionRequest {
        DispatchAction d = new DispatchAction(sub, act, block) ;
        addAction(d) ;
    }

    /// \brief start the ParallelAction.
    /// This in turn calls start() on each of the child actions.  If the policy for the
    /// ParallelAction is 'First', then if any child action completes in its start() method
    /// this action will complete in its start method.
    @Override
    public void start() throws Exception {
        super.start() ;

        boolean done = false ;
        running_ = true ;

        for(Action act : actions_)
        {
            act.start() ;
            if (act.isDone() && done_policy_ == DonePolicy.First)
                done = true ;
        }

        if (done == true)
            completeAction() ;
    }

    /// \brief run the ParallelAction.
    /// This in turn calls run() on each of its child actions that are not already
    /// done.  This method sets this action to done based on the DonePolicy assigned when
    /// the ParallelAction was created.
    @Override
    public void run() throws Exception {
        super.run() ;

        boolean done  ;
        if (done_policy_ == DonePolicy.All)
        {
            done = true ;
            for(Action act : actions_)
            {
                if (!act.isDone()) {
                    act.run() ;
                    if (!act.isDone())
                        done = false ;
                }
            }
        }
        else
        {  
            done = false ; 
            for(Action act : actions_)
            {
                if (!act.isDone()) {
                    act.run() ;
                    if (act.isDone())
                        done = true ;
                }
                else {
                    done = true ;
                }
            }
        }

        if (done == true)
            completeAction() ;
    }

    /// \brief cancel this ParallelAction.
    /// This method will call cancel on all child actions that are running (not done).
    @Override
    public void cancel() {
        super.cancel() ;

        for(Action act : actions_) {
            if (!act.isDone())
                act.cancel() ;
        }
    }

    /// \brief return a human readable string describing this action
    /// \returns a human readable string describing this action
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "ParallelAction [" ;
        boolean first = true ;

        for(Action act : actions_)
        {
            if (!first)
                ret += "," ;

            ret += "\n" ;
            ret += act.toString(indent + 4) ;

            first = false ;
        }
        ret += "\n" ;
        ret += spaces(indent) + "]" ;
        return ret ;
    }    

    /// \brief put all of the children of this ParallelAction into the list provided
    /// \param output the list for the child actions
    @Override
    public void getAllChildren(List<Action> output) {
        for(Action a : actions_) {
            if (a instanceof ActionGroup)
                ((ActionGroup)a).getAllChildren(output) ;
            output.add(a) ;
        }
    }    

    //
    // Complete the action, canceling any child actions that are still running
    // and setting the state of this parent action to done.
    //
    private void completeAction()
    {
        for(Action act : actions_) {
            if (!act.isDone())
                act.cancel() ;
        }

        setDone() ;
    }
} ;