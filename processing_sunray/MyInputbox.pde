//https://forum.processing.org/one/topic/taking-user-input.html

class Inputbox
{
    float x;
    float y;
    String chars;
    int idx;
    boolean numberOnly;
     
    Inputbox(float x, float y)
    {
        this.x = x;
        this.y = y;
        chars = "";
        idx = 0;
        numberOnly = false;
    }
      

    void display()
    {
        fill(0); 
        //textSize(font);
        text(chars,x,y);
        String subStr = chars.substring(0, idx);
        //println(subStr);
        float sw = textWidth(subStr);
        float ta = textAscent();
        line(x+sw, y-ta, x+sw, y-ta+textAscent());  //eingabecursor
    }
    
    void addChar(char c)
    {
        // Add at the end
        if(idx == chars.length()){
          idx++;
          chars += c;
        }  else {
        //Add in the middle
         String subStr1 = chars.substring(0, idx);
         String subStr2 = chars.substring(idx, chars.length());
         chars = subStr1 + c +subStr2;
         idx++;
        }
        
    }
    
    String readString()
    {
        return chars;
    }
    
    String readStringRN()
    {
        return chars + "\r\n";
    }
   
   
    void reset()
    {
        chars = "";
        idx=0;
    }
    
    void deleteChar()
    {
            if (chars.length() > 0)
            {        
                    // Delete at the end
                    if(idx == chars.length()){
                         chars = chars.substring(0,chars.length()-1);
                        if(idx>chars.length()){
                          idx=chars.length();
                        }

                    }  else {
                    //Delete right to curser
                     String subStr1 = chars.substring(0, idx);
                     String subStr2 = chars.substring(idx+1, chars.length());
                     chars = subStr1 + subStr2;
                    }
            }
    }

    void backspaceChar()
    {
            if (chars.length() > 0)
            {        
                    // Delete at the end
                    if(idx == chars.length()){
                         chars = chars.substring(0,chars.length()-1);
                        if(idx>chars.length()){
                          idx=chars.length();
                        }

                    }  else {
                       if(idx > 0){  
                      //Delete left to curser
                       String subStr1 = chars.substring(0, idx-1);
                       String subStr2 = chars.substring(idx, chars.length());
                       chars = subStr1 + subStr2;
                       idx--;
                       }
                    }
            }
    }


    String keyAnalyzer(char c)
    {
        if (c >= '0' && c <= '9') {
             return "NUMBER";
        }  
        else if (c == '.')
        {
            return "NUMBER";
        }
        if (c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z'){
           return "LETTER";
        }  
        else if (c== ',' || c== '!')
        {
            return "LETTER";
        }
        else
        {
            return "OTHER";
        }
    }

    
    void keyPressed()
    {
       if(numberOnly){
          if (keyAnalyzer(key).compareTo("NUMBER") == 0)
          {
              addChar(key);
          }
       }
       else{
          if (keyAnalyzer(key).compareTo("LETTER") == 0 || keyAnalyzer(key).compareTo("NUMBER") == 0)
          {
              addChar(key);
          }
        }  
        
        if (key == CODED) {
          if (keyCode == LEFT) {
            if(idx>0){
              idx--;
            }  
          } else if (keyCode == RIGHT) {
            if(idx<chars.length()){
              idx++;
            }
          } 
        }
  
        if (keyCode == BACKSPACE)
        {
            backspaceChar();
        }
        if (keyCode == DELETE)
        {
            deleteChar();
        }        
        
    }
}