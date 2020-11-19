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
            
            NodeList paths = doc.getElementsByTagName("Paths");

            Node selectedPath = paths.item(0);
            System.out.println(selectedPath);

            //System.out.println(selectedPath.item(0).getAttributes());


        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
