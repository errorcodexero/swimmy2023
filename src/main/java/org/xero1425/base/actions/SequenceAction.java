package org.xero1425.base.actions ;

import java.util.List;
import java.util.ArrayList;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MessageLogger;

/// \file

/// \brief This action executes a set of child actions in sequence.
/// This action is done when the last of the actions is done.  Note, this class
/// understands child actions that complete in their start() method.  In this case
/// the SequenceAction will move on to the next action.  In this way, multiple child
/// actions can be completed in a single call to start() or run().
public class SequenceAction extends ActionGroup
{
    // The indes of the currently running child action
    private int index_ ;

    // If true, this action is running
    private boolean running_ ;

    // The set of child actions
    private List<Action> actions_ ;

    /// \brief create a new SequenceAction
    /// \param logger the message logger for the robot
    public SequenceAction(MessageLogger logger) {
        super(logger) ;

        actions_ = new ArrayList<Action>() ;
        running_ = false ;
    }

    public Action get(int index) {
        return actions_.get(index) ;
    }

    /// \brief return all child actions of this action
    /// \param output the list to contains the child actions
    @Override
    public void getAllChildren(List<Action> output) {
        for(Action a : actions_) {
            if (a instanceof ActionGroup)
                ((ActionGroup)a).getAllChildren(output) ;
            output.add(a) ;
        }
    }      

    /// \brief add a child action to the end of the action sequence
    /// \param act the action to add to the sequence action.  
    public void addAction(Action act) throws InvalidActionRequest {
        if (running_)
            throw new InvalidActionRequest(this, InvalidActionRequest.Reason.ModifyingRunningAction, 
                    "cannot add actions to sequence after it has been started") ;

        actions_.add(act) ;
    }

    /// \brief add a child action to be assigned to a subsystem to this sequence
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

    /// \brief clear all child actions from this sequence.
    public void clear() throws InvalidActionRequest {
        if (running_)
            throw new InvalidActionRequest(this, InvalidActionRequest.Reason.ModifyingRunningAction, 
                    "cannot clear a running SequenceAction") ;
        index_ = -1 ;
        actions_.clear() ;
    }

    /// \brief return the number of child actions in the SequenceAction
    public int size() {
        return actions_.size() ;
    }

    /// \brief start this action by starting the first action in the sequence.
    @Override
    public void start() throws Exception {
        super.start() ;
        running_ = true ;
        index_ = -1 ;

        startNextAction() ;
    }

    /// \brief run this action by running the current child action.
    /// If it is done, start the next action.  Continue this process until
    /// all child actions are complete.
    @Override
    public void run() throws Exception {
        super.run() ;

        actions_.get(index_).run() ;
        if (actions_.get(index_).isDone())
            startNextAction() ;
    }

    /// \brief cancel this action
    /// This in turn cancels the currently running child action.
    @Override
    public void cancel() {
        if (index_ >= 0 && index_ <= actions_.size())
            actions_.get(index_).cancel() ;

        super.cancel() ;            
    }

    /// \brief return a human readable string describing this action
    /// \returns a human readable string describing this action    
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "Sequence [" ;
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

    //
    // Start the next action.  If there is no next action, set this action to
    // the done state.  This method takes into account if a child action completes in
    // its start method and moves on to the next action.
    //
    private void startNextAction() throws Exception {
        while (index_ < actions_.size())
        {
            index_++ ;
            if (index_ < actions_.size())
            {
                actions_.get(index_).start() ;
                if(!actions_.get(index_).isDone())
                    break ;
            }
            else
            {
                running_ = false ;
                setDone() ;
            }
        }
    }
} ;