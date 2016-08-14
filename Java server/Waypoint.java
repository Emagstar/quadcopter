/**
 * Created by max_m on 13/08/2016.
 */
public class Waypoint implements java.io.Serializable
{
    private double longitude;
    private double latitude;
    private double height;


    public Waypoint(double longitude, double latitude, double height)
    {
        this.longitude = longitude;
        this.latitude = latitude;
        this.height = height;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getHeight() {
        return height;
    }

    public void setHeight(double height) {
        this.height = height;
    }
}
