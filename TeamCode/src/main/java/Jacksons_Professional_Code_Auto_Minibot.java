import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Littlekid_Auto_Minibot;
@Autonomous(name = "Weird Spinny", group = "")
public class Jacksons_Professional_Code_Auto_Minibot extends Littlekid_Auto_Minibot{
    @Override
    public void runAuto() {
        setDrive(1.0, 1.0);
        //del *.*
        sleep(3001);

        setDrive(-1.0,1.0);

        sleep(1001);

        setDrive(1.0, 1.0);

        sleep(1000);

        setDrive(1.0, -1.0);

        sleep(400);

        setDrive(0.0, -1.0);

        sleep(400);

        setDrive(1.0, 0.0);

        sleep(400);

        setDrive(-1.0, -0.5);

        sleep(300);
        
        setDrive(0.0, 0.0);

        sleep(100);

        setDrive(0.1, 0.1);

        sleep(100);

        setDrive(0.2, 0.2);

        sleep(100);

        setDrive(0.3, 0.3);

        sleep(100);

        setDrive(0.4, 0.4);

        sleep(100);

        setDrive(0.5, 0.5);

        sleep(100);

        setDrive(0.6, 0.6);

        sleep(100);

        setDrive(0.7, 0.7);

        sleep(100);

        setDrive(0.8, 0.8);

        sleep(100);

        setDrive(0.9, 0.9);

        sleep(100);

        setDrive(1.0, 1.0);

        sleep(100);

        setDrive(-1.0, -1.0);

        sleep(2000);

        setDrive(1.0, 1.0);

        sleep(2000);


    }
}
