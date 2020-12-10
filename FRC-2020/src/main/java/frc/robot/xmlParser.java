package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import javax.xml.parsers.*;
import org.w3c.dom.*;

public class xmlParser {
    public static void main(String args[]) {
        try {
            File robotFile = new File("Auto.xml"); //move to deploy once done
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setIgnoringElementContentWhitespace(true);
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(robotFile);
            
            List<Node> commandGroups = removeWhitespaceNodes(doc.getElementsByTagName("CommandGroup"));

            Node selectedCommandGroup = commandGroups.get(0);

            List<Node> commands = removeWhitespaceNodes(selectedCommandGroup.getChildNodes());
            Node selectedCommand = commands.get(0);

            NamedNodeMap attributes = selectedCommand.getAttributes();

            for(int i = 0; i < attributes.getLength(); i++) {
                System.out.println(attributes.item(i));
            }

            

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static boolean isWhitespaceNode(Node n) {
        if (n.getNodeType() == Node.TEXT_NODE) {
            String val = n.getNodeValue();
            return val.trim().length() == 0;
        } else {
            return false;
        }
    }

    private static List<Node> removeWhitespaceNodes(NodeList nodes) {
        List<Node> validNodes = new ArrayList<Node>();
        for(int i = 0; i < nodes.getLength(); i++) {
            if (!isWhitespaceNode(nodes.item(i)))
                validNodes.add(nodes.item(i));
        }
        return validNodes;
    }

}
