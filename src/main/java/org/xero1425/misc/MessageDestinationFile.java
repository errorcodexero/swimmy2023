package org.xero1425.misc;

import java.io.File;
import java.io.FileWriter;

/// \file

/// \brief a message destination that stores the logged messages in a file
public class MessageDestinationFile implements MessageDestination
{
    private String filename_;
    private FileWriter file_ ;
    private boolean valid_ ;

    /// \brief create a new destination writing to the filename given
    /// \param filename the name of the output file for messages
    public MessageDestinationFile(final String filename) {
        final File f = new File(filename);
        try {
            file_ = new FileWriter(f) ;
            valid_ = true ;
        }
        catch(final Exception ex) {
            valid_ = false ;
            System.err.println("cannot open log file '" + filename_ + "' - " + ex.getMessage()) ;
        }
    }

    /// \brief display a message by appending it to a file
    /// \param type the message type
    /// \param subsystem the subsystem ID for the message
    /// \param msg the text of the message
    public void displayMessage(final MessageType type, final int subsystem, final String msg) {
        if (valid_) {
            try {
                file_.write(msg) ;
                file_.write("\n") ;
                file_.flush() ;
            }
            catch(final Exception ex) {
                System.err.println("cannot write to log file '" + filename_ + "' - " + ex.getMessage()) ;
            }
        }
    }
}
