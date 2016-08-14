import java.awt.*;

/**
 * Created by max_m on 13/08/2016.
 */
public class Driver
{
   public static void main(String[] args)
   {




        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run()
            {
                GUI gui = new GUI();

            }
        });






    }
}
