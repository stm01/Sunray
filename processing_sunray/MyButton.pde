// Built with Processing and Processing.js
// https://hadamlenz.wordpress.com/2014/07/16/building-a-button-in-processing/
class Button extends View {
  String label;
  
  Button( View aparent, int tx, int ty, int tw, int th, String tlabel){
    super(aparent, tx, ty, tw, th);    
    label = tlabel;
  }
  
  void update(){
    super.update();
    //textSize(14); 
    text(label, getXPos()+wid/2-(textWidth(label)/2), getYPos()+floor(hei/2)+floor((textAscent()/2))); 
    //all of this just centers the text in the box
    //println(ypos);
    //println(hei);
    //println(textAscent());
    //println("---");
    
  } 
}