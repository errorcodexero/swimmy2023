package org.xero1425.misc;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

public class JsonReader {
    
    /// \brief read a json settings file
    /// This method returns true if the file was read successfully.  It returns false if
    /// the file failed to read and also prints an error to the logfile.
    /// \param filename the name of the file to read
    /// \returns true if the file was read sucessfully, otherwise false
    public static JSONObject readFile(String filename, MessageLogger logger) {
    
        byte[] encoded;
        try {
            encoded = Files.readAllBytes(Paths.get(filename));
        } catch (IOException e) {
            logger.startMessage(MessageType.Error);
            logger.add("cannot read settings file ").addQuoted(filename).add(" - ");
            logger.add(e.getMessage()).endMessage();
            return null;
        }

        String fulltext = removeComments(encoded) ;

        Object obj = JSONValue.parse(fulltext);
        if (!(obj instanceof JSONObject)) {
            logger.startMessage(MessageType.Error);
            logger.add("cannot read settings file ").addQuoted(filename).add(" - ");
            logger.add("file does not contain a JSON object").endMessage();
            return null;
        }

        return (JSONObject) obj;
    }
    
    static private String removeComments(byte [] encoded) {
        boolean incomment = false ;
        StringBuilder sb = new StringBuilder(encoded.length) ;

        for(int i = 0 ; i < encoded.length ; i++) {
            char ch = (char)encoded[i] ;
            if (incomment) {
                if (ch == '\n') {
                    incomment = false ;
                    sb.append('\n') ;
                }
            }
            else if (i < encoded.length - 1 && ch == '/' && (char)(encoded[i + 1]) == '/') {
                incomment = true ;
            }
            else {
                sb.append((char)encoded[i]) ;
            }
        }

        String str =  sb.toString() ;
        return str ;
    }
}
