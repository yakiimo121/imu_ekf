import hypermedia.net.*;

UDP udp;

final String IP = "127.0.0.1";
final int PORT = 4002;

float r11, r12, r13, r21, r22, r23, r31, r32, r33;
String msg = "";

void setup()
{
  fullScreen(P3D);
  udp = new UDP(this, PORT);
  udp.listen( true );
}

void receive( byte[] data, String ip, int port ) {
  msg = new String( data );
  println(msg);
  String[] datas = msg.split(",");
  r11 = float(datas[0]);
  r12 = float(datas[1]);
  r13 = float(datas[2]);
  r21 = float(datas[3]);
  r22 = float(datas[4]);
  r23 = float(datas[5]);
  r31 = float(datas[6]);
  r32 = float(datas[7]);
  r33 = float(datas[8]);
}

void draw()
{
  background(255); // set background to white
  lights();

  translate(width/2, height/2); // set position to centre

  pushMatrix(); // begin object

  applyMatrix( -1, 0, 0, 0,
               0, 0, 1, 0,
               0, 1, 0, 0,
               0, 0, 0, 1);
               
  applyMatrix( r11, r12, r13, 0,
               r21, r22, r23, 0,
               r31, r32, r33, 0,
               0, 0, 0, 1);

  drawArduino();

  popMatrix(); // end of object
}


void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(200, 400, 20); // draw Arduino board base shape
}
