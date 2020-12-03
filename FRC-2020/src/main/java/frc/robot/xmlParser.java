package frc.robot;

import java.io.File;
import javax.xml.parsers.*;
import org.w3c.dom.*;

public class xmlParser {
    public static void main(String args[]) {
        try {
            File robotFile = new File("Auto.xml");
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setIgnoringElementContentWhitespace(true);
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(robotFile);
            
            NodeList paths = doc.getElementsByTagName("Path");
            System.out.println(paths.getLength()); //currently prints 1

            Node selectedPath = paths.item(0);
            System.out.println(selectedPath); //currently prints [Path: null]

            System.out.println(selectedPath.getAttributes().item(0)); //currently prints ID="HighMode"



        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
