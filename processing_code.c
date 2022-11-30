import processing.serial.*;

Serial myPort;  // Create object from Serial class

void setup() 
{
  size(200,200); //make our canvas 200 x 200 pixels big
  println(Serial.list());
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
}
void draw() 
{
  
  int count=0;
  if (keyPressed == true) 
  {   
    if (key=='d')
  { myPort.write("d");         
   println(key);  
   delay(100);
  }
  if (key=='s')
  { myPort.write("s");         
   println(key);  
   delay(100);
  }
  if (key=='a')
  { myPort.write("a");         
   println(key);  
   delay(100);
  }
  if (key=='w')
  { myPort.write("w");         
   println(key);  
   delay(100);
  }
  if (key=='i')
  { myPort.write("i");         
   println(key);  
   delay(100);
  }
  if (key=='k')
  { myPort.write("k");         
   println(key);  
   delay(100);
  }
  if (key=='j')
  { myPort.write("j");         
   println(key);  
   delay(100);
  }
  if (key=='l')
  { myPort.write("l");         
   println(key);  
   delay(100);
  }
  if (key=='c')
  { myPort.write("c");         
   println(key);  
   delay(100);
  }
  } 
  else 
  println("__INVALID/NO INPUT__");
}