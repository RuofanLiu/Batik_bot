import processing.serial.*;
import java.io.*;
import java.util.*;

int counter = 0;  //increase by one at each iteration in draw();
String subtext;
Serial myPort;
String val;
ArrayList<String> dataList = new ArrayList<String>();  //this is a list that stores the data read, and then send it to arduino
String maxScale;  //this is the variable that determines the scale of the painting, supposed to be type int but is send as type string
boolean firstContact = false;



void setup() {
  //Open the serial port for communication with the Arduino
  //Make sure the COM port is correct
  myPort = new Serial(this, "/dev/cu.usbmodem14601", 9600);
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
      }
      else {            //THIS ELSE STATEMENT IS FOR TEST ONLY
        println(val);
      }
    }
  }
}

void draw() {
  
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
