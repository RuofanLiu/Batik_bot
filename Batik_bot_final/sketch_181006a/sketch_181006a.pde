import processing.serial.*;
import java.io.*;
import java.util.*;
import controlP5.*;

int counter = 0;  //increase by one at each iteration in draw();
String subtext;
Serial myPort;
String val;
ArrayList<String> dataList = new ArrayList<String>();  //this is a list that stores the data read, and then send it to arduino
String maxScale;  //this is the variable that determines the scale of the painting, supposed to be type int but is send as type string
boolean firstContact = false;
ControlP5 cp5;
DropdownList d1;
String portName;
Textarea myTextarea;
Button mybutton;

void setup() {
  clear();
  size(480, 320);
  cp5 = new ControlP5(this);
  //Open the serial port for communication with the Arduino
  //Make sure the COM port is correct
  d1 = cp5.addDropdownList("Select Port")
    .setPosition(16, 100)
    .setSize(168, 200)
    .setHeight(210)
    .setItemHeight(20)
    .setBarHeight(20)
    .setColorBackground(color(60))
    .setColorActive(color(255, 128))
    ;
  myTextarea = cp5.addTextarea("txt")
    .setPosition(200, 20)
    .setSize(250, 280)
    .setFont(createFont("arial", 12))
    .setLineHeight(10)
    .setColor(color(255))
    .setColorBackground(color(255, 100))
    .setColorForeground(color(255, 100));
  ;
  
  cp5.addButton("Select CSV file")
    .setValue(0)
    .setPosition(16, 20)
    .setSize(168, 100)
    ;
  
  d1.getCaptionLabel().set("PORT"); //set PORT before anything is selected

  portName = Serial.list()[1]; //0 as default
  myPort = new Serial(this, portName, 9600);
  //myPort = new Serial(this, "/dev/cu.usbmodem14601", 9600);
  myPort.bufferUntil('\n');
  readData("Desktop/Arduino_shit/Batik_bot_final/test2.csv");
  dataList.add(null);
}

void serialEvent(Serial myPort) {
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  val = myPort.readStringUntil('\n');
  //make sure our data isn't empty before continuing
  if (val != null) {
    //trim whitespace and formatting characters (like carriage return)
    val = trim(val);
    //look for our 'ACK' string to start the handshake
    //if it's there, clear the buffer, and send a request for data
    if (firstContact == false) {
      if (val.equals("ACK")) {
        myPort.clear();
        firstContact = true;
      }
    } else { //if we've already established contact, keep getting and parsing data

      if (val.equals("ACK")) {
        if (counter < dataList.size()) {
          myPort.write(dataList.get(counter));        //send a 1
          counter++;
        }
      } else if (val.equals("DONE")) {
        System.out.println("Job is done. Closing serial connection...");
        myPort.clear();
        myPort.stop();
      } else {            //THIS ELSE STATEMENT IS FOR TEST ONLY
        println(val);
        myTextarea.append(val);
        myTextarea.append("\n");
      }
    }
  }
}

void draw() {
  background(128);

  if (d1.isMouseOver()) {
    d1.clear(); //Delete all the items
    for (int i=0; i<Serial.list().length; i++) {
      d1.addItem(Serial.list()[i], i); //add the items in the list
    }
  }
} 

void controlEvent(ControlEvent theEvent) { //when something in the list is selected
  myPort.clear(); //delete the port
  myPort.stop(); //stop the port
  if (theEvent.isController() && d1.isMouseOver()) {
    portName = Serial.list()[int(theEvent.getController().getValue())]; //port name is set to the selected port in the dropDownMeny
    myPort = new Serial(this, portName, 9600); //Create a new connection
    println("Serial index set to: " + theEvent.getController().getValue());
    delay(2000);
  }
}
/* The following function will read from a CSV or TXT file */
void readData(String myFileName) {

  File file=new File(myFileName);
  BufferedReader br = null;

  try {
    br=new BufferedReader(new FileReader(file));
    String text=null;

    /* keep reading each line until you get to the end of the file */
    while ((text=br.readLine())!=null) {
      subtext = text;
      subtext+='\n';
      dataList.add(subtext);
    }
  }
  catch(FileNotFoundException e) {
    e.printStackTrace();
  }
  catch(IOException e) {
    e.printStackTrace();
  }
  finally {
    try {
      if (br != null) {
        br.close();
      }
    } 
    catch (IOException e) {
      e.printStackTrace();
    }
  }
  //assume that first value of the last row is the maxscale, since the datalist is sorted in this order
  String[] token = dataList.get(dataList.size() - 1).split(",");
  maxScale = token[0];
  maxScale += "\n";
  dataList.add(0, maxScale);  //specify that the first element sent was the scale instead of the coordinate
}
