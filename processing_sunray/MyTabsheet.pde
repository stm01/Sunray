// Built with Processing and Processing.js
// https://hadamlenz.wordpress.com/2014/07/16/building-a-button-in-processing/

class Sheet extends View{
  String label;
  Button button = null;
  
  Sheet( View aparent, String tlabel){    
    super(aparent, 0,28,0,0);     
    label = tlabel;
    Tabsheet tab = (Tabsheet)aparent;    
    button = new Button(this, tab.nextButtonX, -28, 50, 26, tlabel);
    tab.nextButtonX += 50;
    tab.activeSheet = this;
  }
  
  void update(){    
    //textSize(14); 
    //text(label, xpos+wid/2-(textWidth(label)/2), ypos+floor(hei/2)+floor((textAscent()/2))); 
    //all of this just centers the text in the box
    //println(ypos);
    //println(hei);
    //println(textAscent());
    //println("---");
    
    Tabsheet tab = (Tabsheet)parent;
    button.update();
    if (button.clicked) tab.activeSheet = this;
    if (tab.activeSheet != this) return;
    super.update();              
  }   
}


// -------------------------------------------------------------------------------------

class Tabsheet extends View{
  
  Sheet activeSheet = null;
  int nextButtonX = 0;  
   
  Tabsheet( View aparent, int tx, int ty, int tw, int th){
    super(aparent, tx, ty, tw, th);    
  }
    
  void update(){
    super.update();        
        
    //all of this just centers the text in the box
    //println(ypos);
    //println(hei);
    //println(textAscent());
    //println("---");
    
  } 
}