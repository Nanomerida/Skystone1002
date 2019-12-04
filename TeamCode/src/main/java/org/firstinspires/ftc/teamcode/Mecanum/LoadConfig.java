package org.firstinspires.ftc.teamcode.Mecanum;



import java.io.File;
import java.io.IOException;
import java.util.List;

import org.jdom2.Attribute;
import org.jdom2.Document;
import org.jdom2.Element;
import org.jdom2.JDOMException;
import org.jdom2.input.SAXBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


public class LoadConfig {


    public File getConfigFile(String filename)
    {
        File file = new File(filename);
        if (!file.isAbsolute())
        {
            AppUtil.getInstance().ensureDirectoryExists(AppUtil.CONFIG_FILES_DIR);
            file = new File(AppUtil.CONFIG_FILES_DIR, filename);
        }
        return file;
    }


    private Telemetry telemetry;
    File configFile;
    SAXBuilder saxBuilder;
    Document config;
    public DriverConfig.Driver driver;
    public DriverConfig.Manipulator manipulator;

    public LoadConfig(Telemetry telemetry) {
        this.telemetry = telemetry;
        configFile = getConfigFile("robotConfig.xml");
        saxBuilder = new SAXBuilder();

    }


    public void loadAndGetConfig(String driverName, String manipulatorName) {

        try {
            config = saxBuilder.build(configFile);
            Element rootElement = config.getRootElement();

            Element drivers = rootElement.getChild("Drivers");

            List<Element> driverList = drivers.getChildren("Driver");
            List<Element> manipulatorList = drivers.getChildren("Manipulator");

            //Find Driver
            for(Element driver : driverList) {
                if(driver.getAttributeValue("name").equals(driverName)){
                    this.driver = new DriverConfig.Driver(driver.getChildren());
                    break;
                }
            }
            //Find Manipulator
            for(Element manipulator : manipulatorList){
                if(manipulator.getAttributeValue("name").equals(manipulatorName)){
                    this.manipulator = new DriverConfig.Manipulator(manipulator.getChildren());
                    break;
                }
            }


        } catch (IOException f){
            telemetry.addData("oooooooops", "Loading failed!");
            telemetry.update();
        } catch (JDOMException e){
            telemetry.addData("ooooooops", "Loading failed!");
            telemetry.update();
        }
    }

}


