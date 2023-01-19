package org.xero1425.misc;

import java.util.Map;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser ;
import org.apache.commons.csv.CSVRecord ;

/// \file

/// \brief This class loads an projects all of the paths stored in path files.
/// A path is loaded given a name.  The paths directory is searched for a set of
/// files that have the given name as the base name.  These files are loaded and
/// the path can be retreived using the name provided at any future time.
///
public class XeroPathManager
{
    //
    // The type of paths to load and store
    //
    private XeroPathType path_type_ ;

    //
    // The message logger, for logging path manager related messages to the log file
    //
    private MessageLogger logger_ ;

    //
    // The message ID for the path logger
    //
    private int logger_id_ ;

    //
    // The set of paths loaded into the path manager
    //
    private Map<String, XeroPath> paths_ ;

    //
    // The base directory for finding path files
    //
    private String basedir_ ;

    //
    // The extensions for the path data files
    //
    private String[] exts_ ;

    //
    // The name of the messages for the logger
    //
    static final private String LoggerName = "pathmanager" ;

    /// \brief create the path manager
    /// \param logger the message logger
    /// \param basedir the base directory where all paths are found
    /// \param type the type of path algorithm used
    public XeroPathManager(MessageLogger logger, String basedir, XeroPathType type) {
        path_type_ = type ;
        basedir_ = basedir ;
        paths_ = new HashMap<String, XeroPath>() ;
        logger_id_ = logger.registerSubsystem(LoggerName) ;
        logger_ = logger ;

        if (path_type_ == XeroPathType.TankPathFollowing)
        {
            exts_ = new String[2] ;
            exts_[0] = "_left.csv" ;
            exts_[1] = "_right.csv" ;
        }
        else if (path_type_ == XeroPathType.TankPurePursuit)
        {
            exts_ = new String[1] ;
            exts_[0] = "_main.csv" ;
        }
        else if (path_type_ == XeroPathType.TankRamsete)
        {
            exts_ = new String[1] ;
            exts_[0] = "_main.csv" ;
        }
        else if (path_type_ == XeroPathType.SwervePathFollowing)
        {
            exts_ = new String[4] ;
            exts_[0] = "_fl.csv" ;
            exts_[1] = "_fr.csv" ;
            exts_[2] = "_bl.csv" ;
            exts_[3] = "_br.csv" ;
        }
        else if (path_type_ == XeroPathType.SwerveHolonomic) 
        {
            exts_ = new String[1] ;
            exts_[0] = "_main.csv" ;
        }
    }

    /// \brief return the types of paths the path manager is managing
    /// \returns the path type the path manager is managing
    public XeroPathType getPathType() {
        return path_type_ ;
    }

    /// \brief return the extensions for the various data files needed to load
    /// The exts argument must contain an array of strings that is the correct size
    /// based on the XeroPathType given when the path manager was created.  For 
    /// XeroPathType.Swerve this must be four.  For XeroPathType.Tank this must be
    /// two.  For XeroPathType.Robot this must be one.
    /// \param exts the array of extensions
    public boolean setExtensions(String [] exts) {
        if (exts.length != exts_.length)
        {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("setExtensions called with exts array equal to [") ;
            for(int i = 0 ; i < exts.length ; i++)
            {
                if (i != 0)
                    logger_.add(", ") ;
                logger_.add(exts[i]) ;
            }
            logger_.add("], expected ").add(exts_.length).add(" entries") ;
            logger_.endMessage();    
            return false ;
        }

        exts_ = exts ;
        return true ;
    }

    /// \brief return the base directory for the path manager
    /// \returns the base directory for the path manager
    public String getBaseDir() {
        return basedir_ ;
    }

    /// \brief load a path from the path data files
    /// The path manager will look for two files named BASEDIR/name.left_ext and BASEDIR/name.right_ext
    /// where BASEDIR is the base directory specified when the path manager was created, name is the
    /// name given in thie call, and left_ext and right_ext are the extensions set in the setExtensions()
    /// call.
    /// \param name the name of the path to load
    public boolean loadPath(String name) throws Exception {
        String filename = null ;
        Reader [] rdrs = null ;
        CSVParser [] parsers = null ;

        rdrs = new Reader[exts_.length] ;
        parsers = new CSVParser[exts_.length] ;

        for(int i = 0 ; i < exts_.length ; i++)
        {
            try {
                filename = basedir_ + "/" + name + exts_[i];
                rdrs[i] = Files.newBufferedReader(Paths.get(filename)) ;
            }
            catch(Exception ex) {
                logger_.startMessage(MessageType.Error) ;
                logger_.add("cannot load path file (left) '").add(filename).add("' - ").add(ex.getMessage()) ;
                logger_.endMessage();
                return false ;
            }
            try {
                parsers[i] = new CSVParser(rdrs[i], CSVFormat.DEFAULT) ;
            }
            catch(Exception ex) {
                for(int j = 0 ; j < i ; j++)
                {
                    try {
                        parsers[j].close() ;
                        rdrs[j].close() ;
                    }
                    catch(Exception ex2) {
                    }
                }
                logger_.startMessage(MessageType.Error) ;
                logger_.add("cannot load path '").add(name).add("' - ").add(ex.getMessage()) ;
                logger_.endMessage();                
            }
        }

        XeroPath path = new XeroPath(name, exts_.length) ;
        boolean first = true ;
        ArrayList<Iterator<CSVRecord>> iters = new ArrayList<Iterator<CSVRecord>>() ;

        for(int i = 0 ; i < parsers.length ; i++)
        {
            iters.add(parsers[i].iterator()) ;
        }

        CSVRecord [] recs = new CSVRecord[iters.size()] ;

        while (true)
        {
            boolean hasnext = true ;

            for(int i = 0 ; i < iters.size(); i++)
            {
                if (!iters.get(i).hasNext())
                    hasnext = false ;
            }

            if (!hasnext)
                break ;

            for(int i = 0 ; i < iters.size() ; i++)
            {
                recs[i] = iters.get(i).next() ;
            }

            if (first)
            {
                //
                // Skip the column headers in the first row of the data
                //
                first = false ;
                continue ;
            }

            for(int i = 0 ; i < iters.size() ; i++)
            {
                if (recs[i].size() != 10)
                {
                    logger_.startMessage(MessageType.Error) ;
                    logger_.add("cannot load path '").add(name) ;
                    logger_.add("' - ").add(exts_[i]).add(" file contains invalid number of columns, line") ;
                    logger_.add(recs[i].getRecordNumber()) ;
                    logger_.endMessage();   
                    return false ;
                }
                XeroPathSegment seg ;
                try {
                    seg = parseCSVRecord(recs[i]) ;
                }   
                catch(Exception ex) {
                    logger_.startMessage(MessageType.Error) ;
                    logger_.add("cannot load path '").add(name) ;
                    logger_.add("' - ").add(exts_[i]).add(" file contains invalid floating point number, line") ;
                    logger_.add(recs[i].getRecordNumber()) ;
                    logger_.endMessage();   
                    return false ;
                }

                try {
                    path.addPathSegment(i, seg) ;
                }
                catch(Exception ex)
                {

                }
            }
        }

        boolean hasnext = false ;

        for(int i = 0 ; i < iters.size(); i++)
        {
            if (iters.get(i).hasNext())
                hasnext = true ;
        }

        if (hasnext)
        {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("cannot load path '").add(name) ;
            logger_.add("' - files contains differing number of segments") ;
            logger_.endMessage();   
            return false ;
        }

        logger_.startMessage(MessageType.Debug, logger_id_) ;
        logger_.add("loaded path '").add(name) ;
        logger_.endMessage();    

        paths_.put(name, path) ;
        return true ;
    }

    /// \brief returns a path given the path name
    /// \exception MissingPathException thrown when asking for a path that does not exist, see hasPath()
    /// \param name the name of the path to return
    /// \returns a path given its name
    public XeroPath getPath(String name) throws MissingPathException {
        XeroPath p = paths_.get(name) ;
        if (p == null)
            throw new MissingPathException(name) ;

        return p ;
    }

    /// \brief returns true if the path manager has loaded a path with the name given
    /// \returns true if the path manager has loaded a path with the name given
    public boolean hasPath(String name) {
        return paths_.containsKey(name) ;
    }

    private XeroPathSegment parseCSVRecord(CSVRecord r) throws NumberFormatException {
        double time, x, y, pos, vel, accel, jerk, heading, curv, rot ;

        time = Double.parseDouble(r.get(0)) ;
        x = Double.parseDouble(r.get(1)) ;
        y = Double.parseDouble(r.get(2)) ;
        pos = Double.parseDouble(r.get(3)) ;
        vel = Double.parseDouble(r.get(4)) ;
        accel = Double.parseDouble(r.get(5)) ;
        jerk = Double.parseDouble(r.get(6)) ;
        heading = Double.parseDouble(r.get(7)) ;
        curv = Double.parseDouble(r.get(8)) ;
        rot = Double.parseDouble(r.get(9)) ;

        return new XeroPathSegment(time, x, y, pos, vel, accel, jerk, heading, curv, rot) ;
    }
}
