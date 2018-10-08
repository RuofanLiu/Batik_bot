import processing.serial.*;
import java.io.*;
import java.util.*;
int counter = 0;  //increase by one at each iteration in draw();
String subtext;
Serial myPort;
String val;
ArrayList<String> dataList = new ArrayList<String>();  //this is a list that stores the data read, and then send it to arduino

void setup() {
  //Open the serial port for communication with the Arduino
  //Make sure the COM port is correct
  myPort = new Serial(this, "/dev/cu.usbmodem14601", 9600);
  myPort.bufferUntil('\n');
  dataList.add("0,0");
  dataList.add("0,0");
  readData("Desktop/Batik_bot_final/test.csv");
}

void draw() {
  if (counter < dataList.size()) {
    myPort.write(dataList.get(counter));
    counter++;
  }
  delay(500);  //this is necessary!!

  if (myPort.available() > 0) {
    val = myPort.readStringUntil('\n');
    System.out.println(val);
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
  //for(int i = 0; i < dataList.size(); i++){
  //  System.out.println(dataList.get(i)[0] + " " + dataList.get(i)[1]);
  //  delay(1000);
  //}
}
