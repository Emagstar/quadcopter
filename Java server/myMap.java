import com.teamdev.jxmaps.*;
import com.teamdev.jxmaps.Polygon;
import com.teamdev.jxmaps.swing.MapView;

import javax.swing.*;
import java.awt.*;
import javax.swing.*;
import java.awt.*;

/**
 * Created by max_m on 14/08/2016.
 */
public class myMap extends MapView
{
    Map map;

    public myMap(MapViewOptions options) {


        //super(options);
        setOnMapReadyHandler(new MapReadyHandler() {
            @Override
            public void onMapReady(MapStatus status) {
                // Check if the map is loaded correctly
                if (status == MapStatus.MAP_STATUS_OK) {
                    // Getting the associated map object
                    map = getMap();
                    // Creating a map options object
                    MapOptions mapOptions = new MapOptions();
                    // Creating a map type control options object
                    MapTypeControlOptions controlOptions = new MapTypeControlOptions();
                    // Changing position of the map type control
                    controlOptions.setPosition(ControlPosition.TOP_RIGHT);
                    // Setting map type control options
                    mapOptions.setMapTypeControlOptions(controlOptions);
                    // Setting map options
                    map.setOptions(mapOptions);
                    // Setting the map center
                    map.setCenter(new LatLng(55.2074052,-1.3015595));
                    // Setting initial zoom value
                    map.setZoom(5.0);
                    plotPoints();

                }
            }
        });

    }

    public void plotPoints()
    {
        // Creating a path (array of coordinates) that represents a polygon
        LatLng[] path = {new LatLng(25.774, -80.190),
                new LatLng(18.466, -66.118),
                new LatLng(21.291, -157.821),
                new LatLng(32.321, -64.757),
                new LatLng(25.774, -80.190)};
        // Creating a new polygon object
        Polygon polygon = new Polygon(map);
        // Initializing the polygon with the created path
        polygon.setPath(path);
        // Creating a polyline options object
        PolygonOptions options = new PolygonOptions();
        // Setting fill color value
        options.setFillColor("#FF0000");
        // Setting fill opacity value
        options.setFillOpacity(0);
        // Setting stroke color value
        options.setStrokeColor("#FF0000");
        // Setting stroke opacity value
        options.setStrokeOpacity(0.8);
        // Setting stroke weight value
        options.setStrokeWeight(2.0);
        // Applying options to the polygon
        polygon.setOptions(options);
    }

}
