package com.projecttango.examples.java.pointcloud;

/*Transciever
 * Transmits and receives messages from Server on SunRover
 */

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.LinkedList;
import java.util.Queue;

public class Transciever extends Thread{
    Socket socket;
    PrintWriter out;
    BufferedReader in;
    Queue<String> buffer = new LinkedList<String>();
    boolean good = true;

    public Transciever(final String hostname, final int port) {
        //Try to open a port and wait until connected
        System.out.println("Trying to Connect");
        try {
            new Thread(new Runnable(){
                public void run(){
                    //open socket
                    try {
                        socket = new Socket(hostname, port);
                    } catch (Exception e) {
                        System.out.println("Failed to open socket");
                        e.printStackTrace();
                    }
                }
            }).start();
            while (socket == null) {}
            System.out.println("Made Socket");
            in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            out = new PrintWriter(socket.getOutputStream(), true);
            System.out.println("Connected");
        } catch (IOException e) {
            System.out.println("Couldn't connect");
            e.printStackTrace();
            good = false;
        }
    }

    //State of connection
    public boolean isGood() {
        return good;
    }

    //Send a string to the client
    public void sendMessage(String s) {
        out.println(s);
    }

    //Receive a string over the connection
    public String getMessage() {

        String line = null;

        try {
            line = in.readLine();
        } catch (IOException e) {
            good = false;
            e.printStackTrace();
        }

        if (line == null)
            good = false;

        return line;
    }

    //Read a line from the buffer
    public String readLine() {
        String line;

        synchronized(buffer)  {
            line = buffer.poll();
        }

        return line;
    }

    //Close the connection and port
    public boolean close() {
        try {
            socket.close();
        } catch (IOException e) {
            return false;
        }

        return true;
    }

    //Wait for input
    public void run() {
        String line;

        while (isGood()) {
            line = getMessage();
            if (line != null) {
                buffer.add(line);
                //System.out.println("*" + buffer.size());
            }
        }
    }
}


