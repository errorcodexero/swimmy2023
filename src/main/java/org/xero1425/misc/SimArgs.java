package org.xero1425.misc ;

/// \file

/// \brief simulation arguments that need to be sorted in a global scope
public class SimArgs {
    /// \brief the name of the logfile
    public static String LogFileName = null ;

    /// \brief the name of the input stimulus file
    public static String InputFileName = null ;

    public static void processArgs(String... args) {
        int i = 0 ;
        while (i < args.length) {
            if (args[i].equals("--input")) {
                i++ ;
                if (i == args.length) {
                    System.err.println("command line argument --logfile requires an additional argument") ;
                    System.exit(2) ;                    
                }
                SimArgs.InputFileName = args[i];
            }
            else if (args[i].equals("--logfile")) {
                i++ ;
                if (i == args.length) {
                    System.err.println("command line argument --logfile requires an additional argument") ;
                    System.exit(2) ;                    
                }
                SimArgs.LogFileName = args[i] ;
            }
            else {
                System.err.println("unknown command line argument '" + args[i] + "'") ;
                System.exit(2) ;
            }
    
            i++ ;
        }
    }
} ;
