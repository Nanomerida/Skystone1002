package org.firstinspires.ftc.teamcode.Mecanum;



/* import java.io.File;
import java.io.IOException;
import java.util.List;

import org.jdom2.Attribute;
import org.jdom2.Document;
import org.jdom2.Element;
import org.jdom2.JDOMException;
import org.jdom2.input.SAXBuilder */

public class XMLThing {

/*
try {
  File inputFile = new File("input.txt");
  SAXBuilder saxBuilder = new SAXBuilder();
  Document document = saxBuilder.build(inputFile);
  Element rootElement = document.getRootElement();
  
  Element drivers = rootElement.getChild("Drivers");
  
  List<Element> driverList = drivers.getChildren("Driver");
  
  Element driver1 = driverList.get(0);
  Element driver2 = driverList.get(1);
  
  Attribute name1 = driver.getAttribute("name");
  Attribute name2 = driver.getAttribute("name");
  
  telemetry.addData("Driver Config:", "?");
  telemetry.addData(name1.getValue(), "Press A");
  telemetry.addData(name2.getValue(), "Press B");
  telemetry.update();
  int a;
  while(opModeIsActive()){
    if(gamepad1.a) a = 0;
    if(gamepad1.b) a = 1;
  }
  
  
  
} catch(Exception e) { }


}
