String float2String(float number){
  String s = String.format("%.2f", number);
  s = s.replaceAll(",", ".");
  return s;
}


void drawRobot(int px, int py, float pitch, float roll){    
  noStroke();
  // body
  fill(255, 80, 80);
  //fill(255);
  pushMatrix();
    translate(px, py, 0);
    //rotateY(-PI/180*10);
    pushMatrix();
      rotateZ(pitch);
      rotateX(roll);         
      scale(2, 0.5, 1);
      box(50);
      // tires
      scale(0.5, 2.0, 0.1);
      fill(120, 120, 255);
      pushMatrix();
        translate(20, 10, -250);  
        sphere(30); 
      popMatrix(); 
      translate(20, 10, 250);  
      sphere(30);   
    popMatrix();
    // indicate level axis    
    fill(255, 0, 0);
    pushMatrix();
      scale(250, 1, 1);
      box(1);
    popMatrix();
    fill(0, 0, 255);
    pushMatrix();      
      scale(1, 1, 250);
      box(1);
    popMatrix();
    fill(0, 200, 0);
    pushMatrix();
      scale(1, 200, 1);
      box(1);
    popMatrix();        
  popMatrix();
  fill(0,0,255);  
  text("pitch: "+float2String(pitch/PI*180.0), px+60, py-30);
  fill(255,0,0);
  text("roll: "+float2String(roll/PI*180.0), px+70, py+20);
}


void drawNeedle(int cx, int cy, float course){
  //float yaw = course+PI/2;
  float yaw = course;
  int w = 150;
  int px = Math.round(cos(yaw) * w/2);  
  int py = Math.round(sin(yaw) * w/2);    
  line(cx+px,cy-py, cx-px,cy+py);  
  line(cx+px,cy-py, Math.round(cx + cos(yaw-0.1)*w*0.4), Math.round( cy - sin(yaw-0.1)*w*0.4 ));
  line(cx+px,cy-py, Math.round(cx + cos(yaw+0.1)*w*0.4), Math.round( cy - sin(yaw+0.1)*w*0.4 ));    
}

void drawCompass(int cx, int cy, float yaw, float comYaw){
  int w = 150;
  fill(200, 200, 255);
  stroke(0, 0, 0);  
  strokeWeight(2);
  ellipse(cx, cy, w, w);
  stroke(200,200,200);
  line(cx, cy, cx+w/2, cy);  
  strokeWeight(4);
  stroke(180,180,0);
  drawNeedle(cx,cy,comYaw);
  stroke(0,0,255);
  drawNeedle(cx,cy,yaw);  
  strokeWeight(2);
  translate(0,0,1);
  rect(cx-30,cy-10,60,20,10);
  fill(0,0,0);  
  text(float2String(yaw/PI*180.0), cx-20, cy+5);  
}