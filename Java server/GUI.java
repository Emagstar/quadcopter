import com.teamdev.jxmaps.MapViewOptions;


import javax.swing.*;
import javax.swing.table.TableColumn;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;

/**
 * Created by max_m on 14/08/2016.
 */
public class GUI implements ActionListener
{
    private myMap mapView;
    private JFrame f;
    private Connection c = new Connection("192.168.1.60", 2000);


    private String column_Header[] = {"Number","Longitude", "Latitude", "Altitude"};
    private JTable waypointTable = new JTable(1000, 4); //create the table
    private int rowCounter = 0;

    private HashMap<Integer, Waypoint> waypointList = new HashMap<Integer, Waypoint>();


    private Double Latitude = null;
    private Double Longitude = null;
    private Double Altitude = null;


    //global text fields
    private JTextField textField1 = new JTextField();
    private JTextField textField2 = new JTextField();
    private JTextField textField3 = new JTextField();

    public GUI()
    {


        initUI();
    }

    private void initUI()
    {
        GridLayout mainLayout = new GridLayout(2,2);
        GridLayout inputLayout = new GridLayout(4,2);


        JButton sendButton = new JButton("Send!");
        sendButton.setActionCommand("send");
        JButton addButton = new JButton("Add!");
        addButton.setActionCommand("add");


        JPanel inputPannel = new JPanel();
        inputPannel.setLayout(inputLayout);
        String[] inputLabels = {"Longitude:", "Latitude:", "Altitude (in meters):"};
        JLabel l1 = new JLabel(inputLabels[0]);
        JLabel l2 = new JLabel(inputLabels[1]);
        JLabel l3 = new JLabel(inputLabels[2]);
        JLabel l4 = new JLabel("Add to list");


        inputPannel.add(l1);
        inputPannel.add(textField1);
        inputPannel.add(l2);
        inputPannel.add(textField2);
        inputPannel.add(l3);
        inputPannel.add(textField3);
        inputPannel.add(l4);
        inputPannel.add(addButton);



        MapViewOptions options = new MapViewOptions();
        options.importPlaces();
        mapView = new myMap(options);



        f=new JFrame();//creating instance of JFrame


        waypointTable.setFillsViewportHeight(true);

        JScrollPane scrollPane = new JScrollPane(waypointTable);



        f.setTitle("Controls");
        f.setSize(1200, 800);
        f.setLocationRelativeTo(null);
        f.setLayout(mainLayout);


        f.add(inputPannel);
        f.add(mapView);
        f.add(scrollPane);
        f.add(sendButton);

        ;

        for(int i=0;i<waypointTable.getColumnCount();i++)  //set the column headings
        {
            TableColumn column1 = waypointTable.getTableHeader().getColumnModel().getColumn(i);
            column1.setHeaderValue(column_Header[i]);

        }

        f.setResizable(true);//allow the window to be resizeable
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.setVisible(true);


        sendButton.addActionListener(this);
        addButton.addActionListener(this);
    }








    public void actionPerformed(ActionEvent e)
    {
        System.out.println(e.getActionCommand());
        if ("send".equals(e.getActionCommand()))
        {
            //Waypoint w = new Waypoint(Latitude, Longitude, Altitude);
            for(int i = 0;i < waypointList.size();i++)
            {
                c.createConnection();
                c.send(waypointList.get(i));
                System.out.println("Sent!");
                System.out.println("......................................");
            }




        }
        else if ("add".equals(e.getActionCommand()))
        {
            System.out.println("Adding");
            Latitude = Double.parseDouble(textField1.getText());
            Longitude = Double.parseDouble(textField2.getText());
            Altitude = Double.parseDouble(textField3.getText());
            waypointTable.setValueAt(rowCounter,rowCounter,0);
            waypointTable.setValueAt(Latitude,rowCounter,1);
            waypointTable.setValueAt(Longitude,rowCounter,2);
            waypointTable.setValueAt(Altitude,rowCounter,3);
            mapView.plotPoints();
            Waypoint w = new Waypoint(Latitude,Longitude,Altitude);

            waypointList.put(rowCounter,w);

            rowCounter++;
        }
    }

}
