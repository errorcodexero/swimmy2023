package org.xero1425.websrv;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;

public class StatusServer extends Thread {
    private int port_ ;
    private ServerSocket socket_ ;
    private String robot_status_ ;
    private Map<String, String> subsystem_status_ ;

    public StatusServer(int port) throws IOException {
        port_ = port ;
        subsystem_status_ = new HashMap<String, String>() ;
    }

    public synchronized void setRobotStatus(String st) {
        robot_status_ = st ;
    }

    public synchronized void setSubsystemStatus(String subsys, String st) {
        subsystem_status_.put(subsys, st) ;
    }

    public void run() {
        try {
            socket_ = new ServerSocket(port_) ;
        }
        catch(IOException ex) {
            return ;
        }

        while (true) {
            Socket client ;

            try {
                client = socket_.accept() ;
            }
            catch(IOException ex) {
                client = null ;                
            }

            if (client == null) {
                try {
                    socket_.close() ;
                }
                catch(IOException ex) {
                }
                break ;
            }

            try {
                DataOutputStream strm = new DataOutputStream(client.getOutputStream()) ;
                String st = "<html><body>" + robot_status_ ;
                for(String subsys : subsystem_status_.keySet()) {
                    String status = subsystem_status_.get(subsys) ;
                    st += "<hr><h2>Subsystem: " + subsys + "</h2>" ;
                    st += status ;
                }

                strm.writeBytes("HTTP/1.1 200 OK\r\n") ;
                strm.writeBytes("Content-Length: " + Integer.toString(st.length()) + "\r\n") ;
                strm.writeBytes("Content-Type: text/html; charset=utf-8\r\n\r\n") ;
                strm.writeBytes(st) ;
                client.close() ;
            }
            catch(IOException ex) {

            }
        }
    }
}
