package org.xero1425.misc;

import java.io.File;
import java.io.FileWriter;
import java.util.Date;

/// \file

/// \brief This class is a MessageDestination that logs messages to a thumb drive on the roborio
public class MessageDestinationThumbFile implements MessageDestination
{
    //
    // THe name of the file
    //
    private String filename_;

    //
    // The file writer for writing information to the file
    //
    private FileWriter file_ ;

    //
    // If true, the file writer is valid
    //
    private boolean valid_;

    /// \brief create a new object that logs messages to a thumb drive
    /// \param bdir the base directory for log files (e.g. /u) ;
    /// \param timeout a timeout for trying to open a file in the base directory given
    public MessageDestinationThumbFile(final String bdir, final long timeout) {
        int index = 1;
        valid_ = false;
        final long startms = new Date().getTime();

        while (true) {
            final long now = new Date().getTime();
            if (now - startms > timeout) {
                System.err.println("timeout while opening robot log file");
                valid_ = false;
                break;
            }

            final String filename = bdir + "/logfile_" + Integer.toString(index++);
            final File f = new File(filename);
            if (!f.exists())
            {
                valid_ = true ;
                filename_ = filename ;
                try
                {
                    file_ = new FileWriter(f) ;
                }
                catch(final Exception ex)
                {
                    valid_ = false ;
                    System.err.println("cannot open log file '" + filename_ + "' - " + ex.getMessage()) ;
                }
                break ;
            }
        }
    }

    /// \brief display a message by appending it to a file on the thumb drive
    /// \param type the message type
    /// \param subsystem the subsystem ID for the message
    /// \param msg the text of the message
    public void displayMessage(final MessageType type, final int subsystem, final String msg) {
        if (valid_) {
            try
            {
                file_.write(msg) ;
                file_.write("\n") ;
                file_.flush() ;
            }
            catch(final Exception ex)
            {
                System.err.println("cannot write to log file '" + filename_ + "' - " + ex.getMessage()) ;
            }
        }
    }

}
