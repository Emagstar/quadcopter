import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.Socket;

/**
 * Created by max_m on 13/08/2016.
 */
public class Connection
{
    private String ipAddress;
    private int port;
    ObjectOutputStream outToServer;
    ObjectInputStream inFromServer;

    public Connection(String ipAddress, int port)
    {
        this.ipAddress = ipAddress;
        this.port = port;
    }

    public void createConnection()
    {
        // Create the socket
        Socket clientSocket = null;
        try
        {
            clientSocket = new Socket(ipAddress, port);
        }
        catch (IOException e)
        {
            System.out.println("Failed to Create Socket");
            e.printStackTrace();
        }
        // Create the input & output streams to the server
        try
        {
            outToServer = new ObjectOutputStream(clientSocket.getOutputStream());
        } catch (IOException e)
        {
            System.out.println("Failed to Create ObjectOutputStream");
            e.printStackTrace();
        }
        try
        {
            inFromServer = new ObjectInputStream(clientSocket.getInputStream());
        } catch (IOException e)
        {
            System.out.println("Failed to ObjectInputStream");
            e.printStackTrace();
        }
    }

    public void send(Waypoint wp)
    {
        try
        {
            outToServer.writeObject(wp);
        }
        catch (IOException e)
        {
            System.out.println("Failed to Send");
            e.printStackTrace();
        }
    }

}


