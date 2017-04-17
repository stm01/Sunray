// Built with Processing and Processing.js
// https://hadamlenz.wordpress.com/2014/07/16/building-a-button-in-processing/
class View{
  ArrayList<View> children=new ArrayList<View>();
  View parent = null;
  int xpos, ypos, wid, hei;
  boolean over = false;
  boolean down = false; 
  boolean clicked = false;
  
  View(){
  }
  
  View( View aparent, int tx, int ty, int tw, int th){
    parent = aparent;    
    xpos = tx;
    ypos = ty;
    wid = tw;
    hei = th;  
    if (parent != null) parent.addChild(this);
  }
  
  void addChild(View child){
    children.add(child); 
  }
  
  public int getYPos(){
    if (parent == null) return ypos;
      else return parent.getYPos() + ypos;
  }
  
  public int getXPos(){
    if (parent == null) return xpos;
      else return parent.getXPos() + xpos;
  }  
    
  void update(){       
    //it is important that this comes first
    if(down&&over&&!mousePressed){
      clicked=true;
    }else{
      clicked=false;
    }
    
    //UP OVER AND DOWN STATE CONTROLS
    if(mouseX>getXPos() && mouseY>getYPos() && mouseX<getXPos()+wid && mouseY<getYPos()+hei){
      over=true;
      if(mousePressed){
        down=true;
      }else{
        down=false;
      }
    }else{
      over=false;
      down=false; 
    }
    if (children.size() == 0){
      smooth();
    
      //box color controls
      if(!over){
        fill(240);
      }else{
        if(!down){
          fill(151,239,254);
          //fill(255,217,179);
        }else{
          fill(4,114,172);
          //fill(129,208,252);
          //fill(255,174,94);
        }
      }
      stroke(0);
      rect(getXPos(), getYPos(), wid, hei, 5);//draws the rectangle, the last param is the round corners
    
      //Text Color Controls
      if(down){
        fill(255);
      }else{    
        fill(0);
      }
    }
      
    //textSize(14); 
    //text(label, xpos+wid/2-(textWidth(label)/2), ypos+floor(hei/2)+floor((textAscent()/2))); 
    //all of this just centers the text in the box
    //println(ypos);
    //println(hei);
    //println(textAscent());
    //println("---");
        
    for (int i=0; i < children.size(); i++){
      View view = children.get(i);                   
      view.update();      
    }    
    
    
  } 
}