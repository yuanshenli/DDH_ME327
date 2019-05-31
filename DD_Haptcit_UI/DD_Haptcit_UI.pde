
import processing.serial.*;
import controlP5.*;

Serial myPort;        // The serial port

// initialize variables for reading in and mapping
float inByte = 0;
float lastByte = 0;
// State variables
static final int WAITING = 0;
static final int SAMPLEREADY = 1;
static final int SAMPLING = 2;
static final int PLAYBACK = 3;
static final int PLAYBACKREADY = 4;
// communication variables
int lastRequestTime = 0;
int currRequestTime = 0;
int requestInterval = 50;
int currentState = WAITING; //static final 
String ID;
// UI initialization
int offset = 20;
int buttonWidth = 200;
int buttonHeight = 50;
int buttonPositionX = (1200 - 5 * buttonWidth - 4 * offset) / 2;
int buttonPositionY = 200;
int buttonPositionY2 = buttonPositionY + buttonHeight + 2 * offset;
int textWidth = buttonWidth;
int textHeight = buttonHeight;
int textPositionX = buttonPositionX;
int textPositionY = 3 * buttonPositionY;
int fillColorRed = 0;
int fillColorGreen = 128;
int fillColorBlue = 255;
boolean colorDirUpRed = true;
boolean colorDirUpGreen = true;
boolean colorDirUpBlue = false;
int i_dot = 0;
int thresh = 50;

boolean enterPressed = false;

// animation variables
float pos = 0;

ControlP5 cp5;
Button ButtonS1;
Button ButtonS2;
Button ButtonS3;
Button ButtonS4;
Button ButtonS5;
Button ButtonP1;
Button ButtonP2;
Button ButtonP3;
Button ButtonP4;
Button ButtonP5;
Button ButtonReadyS;
Button ButtonReadyP;
Button StopS;
Button StopP;
Textfield TextS1;
Textfield TextS2;
Textfield TextS3;
Textfield TextS4;
Textfield TextS5;

void setup () {

  // set the window size:
  size(1200, 700);    

  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[11], 115200);  //change 11 to number of ports I have
  myPort.bufferUntil('\n');



  addUIElements();
  background(0);      // set inital background:
}

void draw () {
  background(0); //clear background
  updateColor();
  switch(currentState) {
  case WAITING:
    cp5.getTab("default").bringToFront();
    updateWaitingScreen();
    // if sample clicked, have a prompt screen saying to add sample and orient force sensor
    // if prompt screen "Ready!" clicked, send sample/button # to arduino (can send both) and change state to sampling
    break;

  case SAMPLEREADY:
    updateSampleReady();
    break;

  case SAMPLING:
    updateSampling();
    updateAnimation();
    // have a prompt screen that says "sampling..."
    // prompt screen with "done sampling" message, says to name object
    // user presses "ok" and then goes back to waiting state
    
    String inString = "";
    if (myPort.available() > 0) {
      inString = myPort.readStringUntil('\n');
    }
    if (inString.equals("d")) {
      currentState = WAITING;
    }

    break;

  case PLAYBACKREADY:
    updatePlaybackReady();
    break;

  case PLAYBACK:
    updatePlayback();
    updateAnimation();
    break;

  }
}

void updateSampleReady() {
  cp5.getTab("Sampling Prompt").bringToFront();
  textAlign(CENTER);
  textSize(32);
  fill(255);
  text("Before sampling, add sample beneath device finger and orient the force sensor downwards.", width/6, height/6, width - width/3, height/4);
}

void updateSampling() {
  cp5.getTab("Sampling...").bringToFront();
  dotdotdotText("Sampling");       

  textAlign(CENTER);
  textSize(32);
  fill(255);
  text("After sampling, do not forget to name the sample!", width/6, height - height/4, width - width/3, height - height/6);
}

void updatePlaybackReady() {
  cp5.getTab("Playing Back Prompt").bringToFront();
  textAlign(CENTER);
  textSize(32);
  fill(255);
  text("Before playback, remove sample from beneath device finger and orient the force sensor upwards.", width/6, height/6, width - width/3, height/4);
}

void updatePlayback() {
  cp5.getTab("Playback").bringToFront();
  dotdotdotText("Playing back");
}

void dotdotdotText(String myTxt) {
  textAlign(CENTER);
  textSize(32);
  if (i_dot < 5 * thresh && i_dot >= 4 * thresh) {
    text(myTxt + ".....", width/2, height/2 - offset * 4);
    i_dot++;
  } else if (i_dot < 4 * thresh && i_dot >= 3 * thresh) {
    text(myTxt + "....", width/2, height/2 - offset * 4);
    i_dot++;
  } else if (i_dot < 3 * thresh && i_dot >= 2 * thresh) {
    text(myTxt + "...", width/2, height/2 - offset * 4);
    i_dot++;
  } else if (i_dot < 2 * thresh && i_dot >= thresh) {
    text(myTxt + "..", width/2, height/2 - offset * 4);
    i_dot++;
  } else if (i_dot < thresh) {
    text(myTxt + ".", width/2, height/2 - offset * 4);
    i_dot++;
  } else {
    i_dot = 0;
  }
}

void updateAnimation() {
  // request position from Arduino
  currRequestTime = millis();
  if (currRequestTime - lastRequestTime > requestInterval) {
    myPort.write("R");
    lastRequestTime = currRequestTime;
  }
  // Receive position from Arduino
  if (myPort.available() > 0) {
    String inString = myPort.readStringUntil('\n');
    if (inString != null) {
      pos = float(inString);
      println(pos);
    }  
  }
}

void updateWaitingScreen() {
  updateTextAndLines();
  updateButtonLabels();
}

void controlEvent(ControlEvent theControlEvent) {
  if (theControlEvent.isTab()) {
    println("got an event from tab : "+theControlEvent.getTab().getName()+" with id "+theControlEvent.getTab().getId());
  }
  if (theControlEvent.isController()) {

    String name = theControlEvent.getController().getName();

    if (name == "Sample Prompt") {    // READY BUTTON
      myPort.write("s");
      myPort.write(ID); // send the value of the button
      myPort.clear();
      currentState = SAMPLING;
    } else if (name == "StopS") {
      myPort.write("d");
      myPort.clear();
      currentState = WAITING;
    } else if (name == "Playback Prompt") {   // READY BUTTON  
      myPort.write("p");   
      myPort.write(ID); // send the value of the button
      myPort.clear();
      currentState = PLAYBACK;
    } else if (name == "StopP") {
      myPort.write("d");  
      myPort.clear();
      currentState = WAITING;
    } else if (name.indexOf("S") == 0) {
      ID = name.substring(1);
      println(ID);
      currentState = SAMPLEREADY;
    } else if (name.indexOf("P") == 0) {
      println(name);
      ID = name.substring(1);
      println(ID);
      currentState = PLAYBACKREADY;
    }
  }
}

void EventChecker() {
  if (keyPressed) {
    if (key == 's') {
    } else if (key == 'd') {
    } else if (key == 'p') {
    }
  }
}


// checks if enter has been pressed
void keyPressed() {
  if (keyCode == ENTER) {
    enterPressed = true;
  }
} 

void addUIElements() {
  PFont font = createFont("Helvetica", 20, true);
  PFont font1 = createFont("Helvetica", 64, true);
  cp5 = new ControlP5(this);

  cp5.addTab("Sampling Prompt")
    .setColorBackground(color(0, 0, 0))
    .setColorActive(color(0, 0, 0))
    ; 

  cp5.addTab("Sampling...")
    .setColorBackground(color(0, 0, 0))
    .setColorActive(color(0, 0, 0))
    ; 

  cp5.addTab("Playing Back Prompt")
    .setColorBackground(color(0, 0, 0))
    .setColorActive(color(0, 0, 0))
    ; 

  cp5.addTab("Playback")
    .setColorBackground(color(0, 0, 0))
    .setColorActive(color(0, 0, 0))
    ; 

  cp5.getTab("default")
    .activateEvent(true)
    .setCaptionLabel("")
    .setId(1)
    .setColorActive(color(0, 0, 0))
    .setColorForeground(color(0, 0, 0))
    ;

  cp5.getTab("Sampling Prompt")
    .activateEvent(true)
    .setCaptionLabel("")
    .setId(2)
    ;

  cp5.getTab("Sampling...")
    .activateEvent(true)
    .setCaptionLabel("")
    .setId(3)
    ;

  cp5.getTab("Playing Back Prompt")
    .activateEvent(true)
    .setCaptionLabel("")
    .setId(4)
    ;

  cp5.getTab("Playback")
    .activateEvent(true)
    .setCaptionLabel("")
    .setId(5)
    ;

  // buttons and textboxes for the homepage
  ButtonS1 = cp5.addButton("S1")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(0)
    .setPosition(buttonPositionX, buttonPositionY)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonS2 = cp5.addButton("S2")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(1)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonS2.setPosition(buttonPositionX + (int)ButtonS2.getValue() * (buttonWidth + offset), buttonPositionY);

  ButtonS3 = cp5.addButton("S3")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(2)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonS3.setPosition(buttonPositionX + (int)ButtonS3.getValue() * (buttonWidth + offset), buttonPositionY);

  ButtonS4 = cp5.addButton("S4")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(3)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonS4.setPosition(buttonPositionX + (int)ButtonS4.getValue() * (buttonWidth + offset), buttonPositionY);

  ButtonS5 = cp5.addButton("S5")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(4)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonS5.setPosition(buttonPositionX + (int)ButtonS5.getValue() * (buttonWidth + offset), buttonPositionY);

  ButtonP1 = cp5.addButton("P1")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(0)
    .setPosition(buttonPositionX, buttonPositionY2)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonP2 = cp5.addButton("P2")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(1)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonP2.setPosition(buttonPositionX + (int)ButtonP2.getValue() * (buttonWidth + offset), buttonPositionY2);

  ButtonP3 = cp5.addButton("P3")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(2)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonP3.setPosition(buttonPositionX + (int)ButtonP3.getValue() * (buttonWidth + offset), buttonPositionY2);

  ButtonP4 = cp5.addButton("P4")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(3)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonP4.setPosition(buttonPositionX + (int)ButtonP4.getValue() * (buttonWidth + offset), buttonPositionY2);

  ButtonP5 = cp5.addButton("P5")
    .setBroadcast(false)
    .setCaptionLabel("")
    .setValue(4)
    .setSize(buttonWidth, buttonHeight)
    .setFont(font)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setBroadcast(true)
    ;

  ButtonP5.setPosition(buttonPositionX + (int)ButtonP5.getValue() * (buttonWidth + offset), buttonPositionY2);

  TextS1 = cp5.addTextfield("T1")
    .setCaptionLabel("")
    .setPosition(textPositionX, textPositionY)
    .setSize(textWidth, textHeight)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 255, 255))
    .setAutoClear(false)
    ;

  TextS2 = cp5.addTextfield("T2")
    .setCaptionLabel("")
    .setSize(textWidth, textHeight)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 255, 255))
    .setAutoClear(false)
    ;

  TextS2.setPosition(textPositionX + (int)ButtonS2.getValue() * (textWidth + offset), textPositionY);

  TextS3 = cp5.addTextfield("T3")
    .setCaptionLabel("")
    .setSize(textWidth, textHeight)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 255, 255))
    .setAutoClear(false)
    ;

  TextS3.setPosition(textPositionX + (int)ButtonS3.getValue() * (textWidth + offset), textPositionY);

  TextS4 = cp5.addTextfield("T4")
    .setCaptionLabel("")
    .setSize(textWidth, textHeight)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 255, 255))
    .setAutoClear(false)
    ;

  TextS4.setPosition(textPositionX + (int)ButtonS4.getValue() * (textWidth + offset), textPositionY);

  TextS5 = cp5.addTextfield("T5")
    .setCaptionLabel("")
    .setSize(textWidth, textHeight)
    .setFont(font)
    .setFocus(true)
    .setColor(color(255, 255, 255))
    .setAutoClear(false)
    ;

  TextS5.setPosition(textPositionX + (int)ButtonS5.getValue() * (textWidth + offset), textPositionY);

  // buttons for the sampling and playback pages
  ButtonReadyS = cp5.addButton("Sample Prompt")
    .setCaptionLabel("Ready!")
    .setSize(buttonWidth * 2, buttonHeight * 2)
    .setFont(font1)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setPosition(width/2 - buttonWidth, height/2 - buttonHeight);
  ;

  StopS = cp5.addButton("StopS")
    .setCaptionLabel("Stop!")
    .setSize(buttonWidth * 2, buttonHeight * 2)
    .setFont(font1)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setPosition(width/2 - buttonWidth, height/2 - buttonHeight);
  ;

  // buttons for the Playback pages
  ButtonReadyP = cp5.addButton("Playback Prompt")
    .setCaptionLabel("Ready!")
    .setSize(buttonWidth * 2, buttonHeight * 2)
    .setFont(font1)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setPosition(width/2 - buttonWidth, height/2 - buttonHeight);
  ;

  StopP = cp5.addButton("StopP")
    .setCaptionLabel("Stop!")
    .setSize(buttonWidth * 2, buttonHeight * 2)
    .setFont(font1)
    .setColorForeground(color(255, 255, 255))
    .setColorActive(color(230, 230, 230))
    .setPosition(width/2 - buttonWidth, height/2 - buttonHeight);
  ;

  cp5.getController("Sample Prompt").moveTo("Sampling Prompt");
  cp5.getController("StopS").moveTo("Sampling...");
  cp5.getController("Playback Prompt").moveTo("Playing Back Prompt");
  cp5.getController("StopP").moveTo("Playback");
}

void updateColor() {
  if (fillColorRed < 255 && colorDirUpRed == true) {
    fillColorRed = fillColorRed + 1;
  } else if (fillColorRed == 255) {
    colorDirUpRed = false;
    fillColorRed = fillColorRed - 1;
  } else if (fillColorRed == 40) {
    fillColorRed = fillColorRed + 1;
    colorDirUpRed = true;
  } else if (colorDirUpRed == false) {
    fillColorRed = fillColorRed - 1;
  } 

  if (fillColorGreen < 255 && colorDirUpGreen == true) {
    fillColorGreen = fillColorGreen + 1;
  } else if (fillColorGreen == 255) {
    colorDirUpGreen = false;
    fillColorGreen = fillColorGreen - 1;
  } else if (fillColorGreen == 40) {
    fillColorGreen = fillColorGreen + 1;
    colorDirUpGreen = true;
  } else if (colorDirUpGreen == false) {
    fillColorGreen = fillColorGreen - 1;
  } 

  if (fillColorBlue < 255 && colorDirUpBlue == true) {
    fillColorBlue = fillColorBlue + 1;
  } else if (fillColorBlue == 255) {
    colorDirUpBlue = false;
    fillColorBlue = fillColorBlue - 1;
  } else if (fillColorBlue == 40) {
    fillColorBlue = fillColorBlue + 1;
    colorDirUpBlue = true;
  } else if (colorDirUpBlue == false) {
    fillColorBlue = fillColorBlue - 1;
  } 
  color fillColorBackground;
  color fillColorForeground;
  fillColorBackground = color(fillColorRed, fillColorGreen, fillColorBlue);
  fillColorForeground = color(fillColorRed - 40, fillColorGreen - 40, fillColorBlue - 40);

  ButtonS1.setColorBackground(fillColorBackground);
  ButtonS2.setColorBackground(fillColorBackground);
  ButtonS3.setColorBackground(fillColorBackground);
  ButtonS4.setColorBackground(fillColorBackground);
  ButtonS5.setColorBackground(fillColorBackground);
  ButtonP1.setColorBackground(fillColorBackground);
  ButtonP2.setColorBackground(fillColorBackground);
  ButtonP3.setColorBackground(fillColorBackground);
  ButtonP4.setColorBackground(fillColorBackground);
  ButtonP5.setColorBackground(fillColorBackground);
  ButtonReadyS.setColorBackground(fillColorBackground);
  ButtonReadyP.setColorBackground(fillColorBackground);
  StopS.setColorBackground(fillColorBackground);
  StopP.setColorBackground(fillColorBackground);


  ButtonS1.setColorForeground(fillColorForeground);
  ButtonS2.setColorForeground(fillColorForeground);
  ButtonS3.setColorForeground(fillColorForeground);
  ButtonS4.setColorForeground(fillColorForeground);
  ButtonS5.setColorForeground(fillColorForeground);
  ButtonP1.setColorForeground(fillColorForeground);
  ButtonP2.setColorForeground(fillColorForeground);
  ButtonP3.setColorForeground(fillColorForeground);
  ButtonP4.setColorForeground(fillColorForeground);
  ButtonP5.setColorForeground(fillColorForeground);
  ButtonReadyS.setColorForeground(fillColorForeground);
  ButtonReadyP.setColorForeground(fillColorForeground);
  StopS.setColorForeground(fillColorForeground);
  StopP.setColorForeground(fillColorForeground);

  ButtonS1.setColorActive(fillColorBackground);
  ButtonS2.setColorActive(fillColorBackground);
  ButtonS3.setColorActive(fillColorBackground);
  ButtonS4.setColorActive(fillColorBackground);
  ButtonS5.setColorActive(fillColorBackground);
  ButtonP1.setColorActive(fillColorBackground);
  ButtonP2.setColorActive(fillColorBackground);
  ButtonP3.setColorActive(fillColorBackground);
  ButtonP4.setColorActive(fillColorBackground);
  ButtonP5.setColorActive(fillColorBackground);
  ButtonReadyS.setColorActive(fillColorBackground);
  ButtonReadyP.setColorActive(fillColorBackground);
  StopS.setColorActive(fillColorBackground);
  StopP.setColorActive(fillColorBackground);

  if (currentState == WAITING) {
    updateRectangles(fillColorBackground);
  } else {
    updateReadyStopRectangle(fillColorBackground);
  }
}

void updateTextAndLines() {
  textAlign(CENTER); 
  textSize(48);
  text("Sampling Buttons", width/2, buttonPositionY - offset);
  text("Playback Buttons", width/2, buttonPositionY2 + buttonHeight + 3*offset - 5);

  textSize(32);
  fill(255);
  text("Enter Sample Names:", width/2, textPositionY - offset);

  stroke(255);
  strokeWeight(4);
  line(offset, (buttonPositionY + buttonHeight + offset), width - offset, (buttonPositionY + buttonHeight + offset));
  noFill();
  noStroke();
}

void updateRectangles(color fillColor) {
  fill(fillColor);
  rect(buttonPositionX-offset/4 + (int)ButtonS1.getValue() * (textWidth + offset), buttonPositionY-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS2.getValue() * (textWidth + offset), buttonPositionY-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS3.getValue() * (textWidth + offset), buttonPositionY-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS4.getValue() * (textWidth + offset), buttonPositionY-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS5.getValue() * (textWidth + offset), buttonPositionY-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS1.getValue() * (textWidth + offset), buttonPositionY2-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS2.getValue() * (textWidth + offset), buttonPositionY2-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS3.getValue() * (textWidth + offset), buttonPositionY2-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS4.getValue() * (textWidth + offset), buttonPositionY2-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
  rect(buttonPositionX-offset/4 + (int)ButtonS5.getValue() * (textWidth + offset), buttonPositionY2-offset/4, buttonWidth+offset/2, buttonHeight+offset/2, 5);
}

void updateReadyStopRectangle(color fillColor) {
  fill(fillColor);
  rect(width/2 - buttonWidth - offset/2, height/2 - buttonHeight - offset/2, buttonWidth * 2 + offset, buttonHeight * 2 + offset, 10);
}

// if enter has been pressed, clear the active textbox and corresponding button
// .isActive() checks if a Textbox is highlighted
void updateButtonLabels() {
  if (enterPressed) { 
    if (TextS1.isActive()) {
      // get the text from the textbook
      ButtonS1.setLabel(cp5.get(Textfield.class, "T1").getText());
      ButtonP1.setLabel(cp5.get(Textfield.class, "T1").getText());
      //clear the text in the textbox
      TextS1.clear();
    } else if (TextS2.isActive()) {
      ButtonS2.setLabel(cp5.get(Textfield.class, "T2").getText());
      ButtonP2.setLabel(cp5.get(Textfield.class, "T2").getText());
      TextS2.clear();
    } else if (TextS3.isActive()) {
      ButtonS3.setLabel(cp5.get(Textfield.class, "T3").getText());
      ButtonP3.setLabel(cp5.get(Textfield.class, "T3").getText());
      TextS3.clear();
    } else if (TextS4.isActive()) {
      ButtonS4.setLabel(cp5.get(Textfield.class, "T4").getText());
      ButtonP4.setLabel(cp5.get(Textfield.class, "T4").getText());
      TextS4.clear();
    } else if (TextS5.isActive()) {
      ButtonS5.setLabel(cp5.get(Textfield.class, "T5").getText());
      ButtonP5.setLabel(cp5.get(Textfield.class, "T5").getText());
      TextS5.clear();
    }
    enterPressed = false;
  }
}
