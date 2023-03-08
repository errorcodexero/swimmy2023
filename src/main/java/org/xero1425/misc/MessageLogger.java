
package org.xero1425.misc;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.List;
import java.text.DecimalFormat;
import java.util.ArrayList ;

/// \file

/// \brief This class acts a a clearing house for messages to be routed from the robot to a person.
///
/// All subsystem and many of the other complex code modules produce debug messages.  These debug messages
/// are routed through this class and filtered so only messages of interest are seen.  Subsystems or modules
/// register their name with the MessageLogger with registerSubsystem() and a message source handle is returned.
/// When a message is started, a message source handle is provided as well as a message type.  Message types can
/// are specified with the MessageType enum and can be Debug, Info, Warning, Error, and Fatal.  Messages are started,
/// data of various types (include text string) are added to the message and the message is ended.  When the message
/// is ended, the various filters are examined to determine if the message should be sent to the user.  If the answer
/// is yes, the then message is forwareded to any number of MessageDestination derived classes that forward messages
/// to the user.  There are MessageDestination classes that forward messages to standard output, the driver station
/// console, and to a file located on a USB thumb drive on the robo rio.  Any optional time source can be registered
/// with the message logger.  If a time source is registered, each message is tagged with the current time stamp.  
/// The MessageLogger is thread safe and can be used with multiple threads concurrently.
public final class MessageLogger
{
    // The per thread data for the logger
    private Map<Long, ThreadData> per_thread_data_ ;

    // The set of destinations for messages
    private List<MessageDestination> destinations_ ;

    // The set of message types enabled
    private List<MessageType> enabled_types_ ;

    // The time source for messages
    private MessageTimeSource time_src_ ;

    // This is a mapping from subsystem number to subsystem name
    private Map<Integer, String> subsystems_ ;

    // The list of enabled subsytems
    private List<Integer> enabled_subsystems_ ;

    // This is the number for the next subsystem registered
    private int subsystem_index_ ;

    // The lock for the serial number
    private Object lock_ ;

    // the list of subsystem to be enabled if ethey are created
    private List<String> to_be_enabled_ ;

    // the format for the time value
    private DecimalFormat format_ ;

    // Serial number for each message
    static int global_serial_ = 1 ;

    // Number of error messages printed
    private int error_count_ ;

    /// \brief the subsystem value that means there is no subsystem
    public static final int NOSUBSYSTEM = 0 ;

    /// \brief Create a new message logger object
    public MessageLogger()
    {
        subsystems_ = new HashMap<Integer, String>() ;
        subsystem_index_ = 1 ;
        time_src_ = null ;
        destinations_ = new ArrayList<MessageDestination>() ;
        per_thread_data_ = new HashMap<Long, ThreadData>() ;

        enabled_types_ = new ArrayList<MessageType>() ;
        enabled_types_.add(MessageType.Debug) ;
        enabled_types_.add(MessageType.Info) ;
        enabled_types_.add(MessageType.Warning) ;
        enabled_types_.add(MessageType.Error) ;
        enabled_types_.add(MessageType.Fatal) ;

        enabled_subsystems_ = new ArrayList<Integer>() ;
        to_be_enabled_ = new ArrayList<String>() ;

        lock_ = new Object() ;

        format_ = new DecimalFormat("000.0000") ;

        error_count_ = 0 ;
    }

    public void logStackTrace(StackTraceElement [] trace) {
        for(int i = 0 ; i < trace.length ; i++) {
            startMessage(MessageType.Error) ;
            add("    ") ;
            add(trace[i].toString()) ;
            endMessage();
        }
    }

    /// \brief returns the total error count
    /// \returns the total error count
    public int getErrorMessageCount() {
        return error_count_ ;
    }

    /// \brief register a new subsystem with the message logger
    /// \param name the name of the subsystem
    /// \returns the message ID handle for messages
    public int registerSubsystem(final String name) {
        final int index = subsystem_index_++;
        subsystems_.put(index, name);

        if (to_be_enabled_.contains(name)) {
            enableLoggerID(index) ;
            to_be_enabled_.remove(name) ;
        }
        return index;
    }

    /// \brief register a time source to return time to the message logger
    /// \param src an object that can provide time, must be derived from the MessageTimeSource class
    public void setTimeSource(final MessageTimeSource src) {
        time_src_ = src;
    }

    /// \brief clear all message destinations
    public void clear() {
        destinations_.clear();
    }

    /// \brief add a new message destination
    /// \param d the new message destination
    public void addDestination(final MessageDestination d) {
        destinations_.add(d);
    }

    /// \brief enable a given message type
    /// \param mt the message type to enable
    public void enableMessageType(final MessageType mt) {
        if (!enabled_types_.contains(mt))
            enabled_types_.add(mt);
    }

    /// \brief disable a given message type
    /// \param mt the message type to disable
    public void disableMessageType(final MessageType mt) {
        if (enabled_types_.contains(mt))
            enabled_types_.remove(mt);
    }

    /// \brief returns true if a given message type is enabled
    /// \returns true if a given message type is enabled
    public boolean isTypeEnabled(final MessageType mt) {
        return enabled_types_.contains(mt);
    }

    /// \brief enable the messages associated with a given subsystem message hangle
    /// \param handle the handle for a given subsystem
    private void enableLoggerID(final int handle) {
        if (!enabled_subsystems_.contains(handle))
            enabled_subsystems_.add(handle);
    }

    /// \brief disable messages given a logger id
    /// \param handle the handle for a given subsystem
    public void disableLoggerID(final int handle) {
        if (enabled_subsystems_.contains(handle))
            enabled_subsystems_.remove(handle);
    }

    /// \brief returns true if the given logger ID is enabled
    /// \param handle the handle to check to see if its enabled
    /// \returns true if the given logger ID is enabled
    public boolean isLoggerIDEnabled(final int handle) {
        return enabled_subsystems_.contains(handle);
    }

    /// \brief enable messages for a given subsystem
    /// \param name the name of the subsystem to enable
    /// \returns true if the messages are enabled sucessfully
    public boolean enableSubsystem(final String name) {
        boolean ret = false;

        for (final Integer key : subsystems_.keySet()) {
            final String subname = subsystems_.get(key);
            if (subname.equals(name)) {
                enableLoggerID(key);
                ret = true;
                break;
            }
        }

        if (!ret)
        {   
            if (!to_be_enabled_.contains(name))
                to_be_enabled_.add(name) ;
            ret = true ;
        }

        return ret;
    }

    /// \brief disable messages for a given subsystem
    /// \param name the name of the subsystem to disable
    /// \returns true if the messages are disabled sucessfully
    public boolean disableSubsystem(final String name) {
        boolean ret = false;

        for (final Integer key : subsystems_.keySet()) {
            final String subname = subsystems_.get(key);
            if (subname == name) {
                disableLoggerID(key);
                ret = true;
                break;
            }
        }

        return ret;
    }

    /// \brief returns true if the subsystem given is enabled
    /// \param name the name of the subsystem to check for enabled
    /// \returns true if the subsystem given is enabled
    public boolean isSubsystemEnabled(final String name) {
        boolean ret = false;

        for (final Integer key : subsystems_.keySet()) {
            final String subname = subsystems_.get(key);
            if (subname == name) {
                ret = isLoggerIDEnabled(key);
                break;
            }
        }

        return ret;
    }

    /// \brief start a new message
    /// \param mtype the type of message to start
    /// \param subsystem the module or subsystem that is displaying the message
    /// \returns a reference to the message logger object
    public MessageLogger startMessage(final MessageType mtype, final int subsystem) {
        final ThreadData per = getPerThreadData();

        if (per.in_message_) {
            //
            // We have a nested message, someone forgot to close off the current
            // message.
            //
            per.message_.append(" DID NOT CALL ENDMESSAGE, serial = ") ;
            per.message_.append(per.serial_) ;
            endMessage();
        }

        per.serial_ = getSerial() ;
        per.in_message_ = true;
        per.message_ = new StringBuilder(80) ;
        per.type_ = mtype;
        per.subsystem_ = subsystem;
        per.enabled_ = enabled_types_.contains(per.type_) && subsystemEnabled(per.subsystem_) ;

        return this;
    }

    /// \brief start a new message with no subsystem
    /// \param mtype the type of message to start
    /// \returns a reference to the message logger object
    public MessageLogger startMessage(final MessageType mtype) {
        return startMessage(mtype, NOSUBSYSTEM);
    }

    private void outputMessage(final ThreadData per)
    {
        String timestr ;
        String threadstr ;
        String typestr ;
        String substr ;

        if (time_src_ == null) {
            timestr = "???.????";
        } else {
            timestr = format_.format(time_src_.getTime()) ;
        }

        threadstr = ":" + Long.toString(per.id_) ;

        typestr = ": " + per.type_.toString() + ": "  ;

        if (per.subsystem_ == 0)
        {
            substr = "global: ";
        }
        else
        {
            String subname = subsystems_.get(per.subsystem_) ;
            if (subname != null)
                substr = subname + ": " ;
            else
                substr = "missing(" + per.subsystem_ + "): " ;
        }

        StringBuilder bld = new StringBuilder() ;
        for(int i = 0 ; i < timestr.length() + typestr.length() + threadstr.length() ; i++)
            bld.append(' ') ;
        String spaces = bld.toString() ;

        String[] lines = per.message_.toString().split("\n") ;
        boolean first = true ;
        for(String line : lines) {
            String msg ;

            if (first)
                msg = timestr + threadstr + typestr + substr + line ;
            else
                msg = spaces + substr + line ;

            for (final MessageDestination dest : destinations_) {
                dest.displayMessage(per.type_, per.subsystem_, msg);
            }

            first = false ;
        }
    }

    /// \brief ends the current message
    /// This method ends the current message and displays the message if the filter tests
    /// allow the message to be displayed.  The message is displayed by passing to each of
    /// the MessageDestination objects registered.
    public void endMessage() {
        final ThreadData per = getPerThreadData();

        if (!per.in_message_)
            return;

        if (per.message_.length() > 0) {
            if (enabled_types_.contains(per.type_) && subsystemEnabled(per.subsystem_)) {
                if (per.type_ == MessageType.Error)
                    error_count_++ ;
                outputMessage(per) ;
            }
        }

        if (per.type_ == MessageType.Fatal) {
            for (final MessageDestination dest : destinations_) {                
                dest.displayMessage(per.type_, per.subsystem_, "fatal error occurred - code aborting") ;
            }
            System.exit(-2);
        }

        per.message_ = null ;
        per.subsystem_ = 0;
        per.in_message_ = false;
    }

    /// \brief add a string to the current message
    /// \param str the string to add
    /// \returns the MessageLogger object
    public MessageLogger add(final String str) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_ && per.in_message_)
            per.message_.append(str) ;
        return this;
    }

    /// \brief add a name value pair to the message
    /// \param name the name to add
    /// \param value the value to add
    /// \returns the MessageLogger object
    public MessageLogger add(final String name, final double value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.4g", value)) ;
        }

        return this;        
    }

    /// \brief add a name value pair to the message
    /// \param name the name to add
    /// \param value the value to add
    /// \returns the MessageLogger object
    public MessageLogger add(final String name, final float value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.4g", value)) ;
        }

        return this;        
    }

    /// \brief add a name value pair to the message
    /// \param name the name to add
    /// \param value the value to add
    /// \returns the MessageLogger object
    public MessageLogger add(final String name, final int value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = ") ;
            per.message_.append(value) ;
        }

        return this;        
    }  
    
    /// \brief add a name value pair to the message
    /// \param name the name to add
    /// \param value the value to add
    /// \returns the MessageLogger object    
    public MessageLogger add(final String name, final boolean value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = ") ;
            per.message_.append(value) ;
        }

        return this;        
    }  
    
    /// \brief add a name value pair to the message
    /// \param name the name to add
    /// \param value the value to add
    /// \returns the MessageLogger object    
    public MessageLogger add(final String name, final String value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = ") ;
            per.message_.append(value) ;
        }

        return this;        
    }       

    public MessageLogger add(final String name, final Pose2d pose) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", pose.getX()));
            per.message_.append(" ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", pose.getY()));
            per.message_.append(" ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.1f", pose.getRotation().getDegrees()));
        }
        return this;
    }

    public MessageLogger add(final String name, final Translation2d t) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = X: ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", t.getX()));
            per.message_.append(" Y: ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", t.getY()));
        }
        return this;
    }

    public MessageLogger add(final String name, final Translation3d t) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_) {
            per.message_.append(" ") ;
            per.message_.append(name) ;
            per.message_.append(" = X: ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", t.getX()));
            per.message_.append(" Y: ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", t.getY()));
            per.message_.append(" Z: ") ;
            per.message_.append(String.format(java.util.Locale.US, "%.3f", t.getZ()));
        }
        return this;
    }

    /// \brief add a quoted string to a messages
    /// \param str the string to add
    /// \returns the MessageLogger object  
    public MessageLogger addQuoted(final String str) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
        {
            per.message_.append("'") ;
            per.message_.append(str) ;
            per.message_.append("'") ;
        }

        return this;
    }    

    /// \brief add a character to a messages
    /// \param ch the character to add
    /// \returns the MessageLogger object  
    public MessageLogger add(final char ch) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
            per.message_.append(ch) ;

        return this;        
    }

    /// \brief add a integer to a messages
    /// \param value the integer to add
    /// \returns the MessageLogger object     
    public MessageLogger add(final int value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
            per.message_.append(value) ;

        return this;
    }

    /// \brief add a value to a messages
    /// \param value the value to add
    /// \returns the MessageLogger object     
    public MessageLogger add(final long value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
            per.message_.append(value) ;

        return this;
    }

    /// \brief add a value to a messages
    /// \param value the value to add
    /// \returns the MessageLogger object     
    public MessageLogger add(final boolean value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
            per.message_.append(value) ;

        return this;
    }

    /// \brief add a value to a messages
    /// \param value the value to add
    /// \returns the MessageLogger object     
    public MessageLogger add(final double value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
            per.message_.append(String.format(java.util.Locale.US, "%.4f", value)) ;

        return this;
    }

    /// \brief add a value to a messages
    /// \param value the value to add
    /// \returns the MessageLogger object     
    public MessageLogger add(final float value) {
        final ThreadData per = getPerThreadData();
        if (per.enabled_&& per.in_message_)
            per.message_.append(String.format(java.util.Locale.US, "%.4f", value)) ;

        return this;
    }

    static private /* synchronized */ int getSerial() {
        return global_serial_++ ;
    }

    private boolean subsystemEnabled(final int sub) {
        return sub == NOSUBSYSTEM || enabled_subsystems_.contains(sub);
    }

    private ThreadData getPerThreadData() {
        ThreadData per = null;

        synchronized(lock_) {
            final long id = Thread.currentThread().getId();
            if (per_thread_data_.containsKey(id))
            {
                per = per_thread_data_.get(id) ;
            }
            else
            {
                per = new ThreadData() ;
                per.id_ = id ;
                per.in_message_ = false ;
                per_thread_data_.put(id, per) ;
            }
        }

        return per ;
    }

    private class ThreadData
    {
        public long id_ ;
        public boolean in_message_ ;
        public boolean enabled_ ;
        public MessageType type_ ;
        public int subsystem_ ;
        public StringBuilder message_ ;
        public int serial_ ;
    } ;

}