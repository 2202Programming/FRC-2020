package frc.robot.util.misc;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import javax.xml.parsers.*;
import org.w3c.dom.*;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

/**
 * xmlParser - read XML file and create a command group that can be run in auto.
 * Elements are easier to parse than attributes and allow more varied systax at
 * the element level.
 * 
 * 
 * XML Syntax <Command name="fq-classname" /> <!-- No args --> <CommandGroup>
 * <Command name="fq-classname" withTimeout="3.3"> <Arg type="fq-type"
 * value="stringValue") /> <Arg robotDevice="name"/> </Command> ...
 * </CommandGroup>
 * 
 * Usage:
 * 
 * XMLParser myParser = new XMLParser(); Command autoCmd =
 * myParser.parse(filename);
 * 
 * 
 * 
 */
public class xmlParser {

    // Node element names for parsing the file
    static final String CMD = "Command";
    static final String ARG = "Arg";
    static final String SEQ_GRP = "SequenceGrp";
    static final String PARA_GRP = "ParallelGrp";
    static final String RACE_GRP = "RaceGrp";
    // attribute key words
    static final String WITHTIMEOUT = "withTimeout";
    static final String NAME = "name";
    static final String TYPE = "type";
    static final String VALUE = "value";
    static final String ROBOT_DEV = "robotDevice";

    CommandGroupBase m_cmd;

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
        for (int i = 0; i < nodes.getLength(); i++) {
            if (!isWhitespaceNode(nodes.item(i)))
                validNodes.add(nodes.item(i));
        }
        return validNodes;
    }

    public xmlParser() {

    }

    public Command parse(String fileName) {
        Document doc;
        // File commandFile = new File("Auto.xml"); //move to deploy once done
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        factory.setIgnoringElementContentWhitespace(true);
        try {
            DocumentBuilder builder = factory.newDocumentBuilder();
            doc = builder.parse(fileName);
            doc.normalize();
        } catch (Exception e) {
            System.out.println("XML parsing error - " + e.toString());
            return null;
        }
        
        // we got this far, so the DOM is valid, now construct our commands
        m_cmd = CommandGroupBase.sequence(); // start with a sequence containe
        m_cmd.setName(fileName);
        buildCommands(m_cmd, doc);
        return m_cmd;
    }

    public void buildCommands(CommandGroupBase group, Node nodElement) {
        Object[] cmdArray;
        CommandGroupBase newGroup;
        Command cmd;

        NodeList nodes = nodElement.getChildNodes();
        for (int i = 0; i < nodes.getLength(); i++) {

            Node node = nodes.item(i);
            if (node.getNodeType() == Node.ELEMENT_NODE) {
                Element element = (Element) node;
                switch (node.getNodeName()) {
                    case CMD:
                        cmdArray = parseCommand(element);
                        cmd = buildCommand(cmdArray);
                        decorate(cmd, element);
                        if (cmd != null)
                            group.addCommands(cmd);
                        break;

                    case SEQ_GRP:
                        newGroup = CommandGroupBase.sequence();
                        group.addCommands(newGroup);
                        buildCommands(newGroup, element);
                        break;

                    case PARA_GRP:
                        newGroup = CommandGroupBase.parallel();
                        group.addCommands(newGroup);
                        buildCommands(newGroup, element);
                        break;

                    default:
                        // ignore anything we don't know
                        break;
                }

            }
        }

    }

    /**
     * buildCommand() - constructs a single command object via reflections
     * 
     * @param cmdArray array of fqcn + value objects
     * @return constructed Command object
     */

    Command buildCommand(Object[] cmdArray) {
        Command retCmd = null;
        String fqcn= (String) cmdArray[0]; // 0th is the fqdn by convention
        Class<?> cls;
        Object[] args = new Object[cmdArray.length - 1];
        
        System.arraycopy(cmdArray, 1, args, 0, cmdArray.length -1);
        
        try {
            cls = Class.forName(fqcn);
             retCmd = (Command)checkInterfaces(cls.getDeclaredConstructors(), args);
        } catch (Exception e) {
            System.out.println("*** Command construction failed:" + e.getMessage() + " *** \n");
            e.printStackTrace(System.out);
        }
        return retCmd;
    }

    Object checkInterfaces(Constructor<?>[] ctors, Object[] args)
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException {
        Class<?>[] newTypes = new Class<?>[args.length];
        Constructor<?> match = null;
        // check each constructor for a match
        for (Constructor<?> ctor : ctors) {
            Class<?>[] cptypes = ctor.getParameterTypes();
           
            // now check all 
            if (cptypes.length != args.length)  continue;

            // check args for match with given parameters
            for (int i = 0; i < args.length; i++) {
                Class<?> cptype = cptypes[i];
                Class<?> atype = (args[i] != null) ? args[i].getClass() : Object.class;

                if (cptype.isInterface()) {
                    for (Class<?> ifx : atype.getInterfaces()) {
                        if (ifx == cptype) {
                            atype = ifx;
                        }
                    }
                }
                // save the adjusted param in the newParam list
                if (cptype == atype) {
                    newTypes[i] = atype;  //
                    match = ctor;
                    continue;   // keep checking this constructor
                } else {
                    match = null;
                    break; // couldn't make this constructor work
                }
            }
            // if we still have a match, we can quit checking
            if (match != null) break;
        }
        if (match != null) {
            return match.newInstance(args);
        }
        return null;
    }


    void decorate(Command cmd, Element cmdElement) {
        if (cmdElement.hasAttribute(WITHTIMEOUT)) {
            double time = Double.parseDouble(cmdElement.getAttribute(WITHTIMEOUT));
            cmd.withTimeout(time);
        }  
    }

    /**
     * Reflection helpers
     * 
     * Here are a few hints -
     * https://stackoverflow.com/questions/40143568/how-do-i-call-a-constructor-in-java-via-reflection
     * https://stackoverflow.com/questions/5760569/illegalargumentexception-with-constructing-class-using-reflection-and-array-argu
     * https://stackoverflow.com/questions/2408789/getting-class-type-from-string
     * 
     */

    /**
     * getArgs - returns all the arguments needed to instantiate the command
     * represented by this node in the xml doc. Fully qualified command name (fqcn)
     * is first Object in the array returned.
     *
     * @param cmdNode a single command
     * @return Object[] fqcn + arguments
     */

    Object[] parseCommand(Element cmdNode) {
        ArrayList<Object> args = new ArrayList<Object>();

        String fqcn = cmdNode.getAttribute("name");
        args.add(fqcn);

        // Child nodes will all be argument Elements or we ignore them
        NodeList childNodes = cmdNode.getChildNodes();
        // now parse the Arg Element and build array for reflection
        for (int i = 0; i < childNodes.getLength(); i++) {

            Node child = childNodes.item(i);
            if (child.getNodeType() == Node.ATTRIBUTE_NODE)  {
                
                break;
            }
            
            if ((child.getNodeType() == Node.ELEMENT_NODE) &&
                (child.getNodeName() == ARG)) {
                Element c = (Element) child;
                // Type will have type and value attributes
                if (c.hasAttribute(TYPE)) {
                    // <Arg type="fq-type" value="stringValue") />
                    String type = c.getAttribute(TYPE);
                    String value = c.getAttribute(VALUE);
                    args.add(createObject(type, value));
                }

                if (c.hasAttribute(ROBOT_DEV)) {
                    // <Arg robotDevice="name"/>
                    String deviceName = c.getAttribute(ROBOT_DEV);
                    // add the requested robot device to the arg array
                    Object dev = RobotContainer.getDeviceByName(deviceName);
                    args.add(dev);
                }
            }
        }
        return args.toArray();
    }

    /**
     * createObject - returns a object made from the type and value strings
     * 
     * @param type
     * @param value
     * @return useful object or null
     */
    Object createObject(String type, String value) {
        Object retval = null;
        switch (type) {
            case "double":
                retval = Double.parseDouble(value);
                break;
            case "int":
            case "integer":
                retval = Integer.parseInt(value);
                break;

            case "boolean":
                retval = Boolean.parseBoolean(value);
                break;
            default:
                System.out.println("***Bad XML Parse for type:" + type + " and value: " + value + "***\n");
        }

        return retval;
    }

}
