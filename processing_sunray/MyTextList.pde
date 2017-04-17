// Built with Processing and Processing.js
// https://hadamlenz.wordpress.com/2014/07/16/building-a-button-in-processing/
class TextList {
  int posX, posY, r, g, b, ploth, plotw;
  StringList list;
  String label;
  float maxRows = 10;
  float headerHeigth = 10;

  String float2String(float number) {
    String s = String.format("%.2f", number);
    s = s.replaceAll(",", ".");
    return s;
  }

  TextList(  String _label, int _posX, int _posY, int _plotw, int _ploth, int _r, int _g, int _b) {

    label = _label;
    posX = _posX;
    posY = _posY;
    r = _r;
    g = _g;
    b = _b;
    ploth = _ploth;
    plotw = _plotw;
    list = new StringList();
    headerHeigth = textAscent()+6;;
    maxRows = floor((ploth-headerHeigth-2)/(textAscent()+2)); //-2 damit Zeile unten nicht direkt auf Linie angezeigt wird.
  }

  void addData(String value) {  
    list.append( value );
    //if (list.size() > plotw) list.remove(0);
    if (list.size() > maxRows) list.remove(0);
  }

  void update() {  

    int px;
    float py;

    stroke(0, 0, 0);
    fill(255, 255, 255);
    //rect(posX, posY, plotw, ploth, 5);
    py = posY+headerHeigth;
    line(posX, py, posX+plotw, py);
    fill(0);
    //text(label, posX+plotw/2-(textWidth(label)/2), posY+ 2+floor((textAscent()/2)) +textAscent()+4);
    text(label, posX+plotw/2-(textWidth(label)/2), posY+textAscent()+2);

    fill(r, g, b);
    stroke(r, g, b);

    px = posX + 5;

    for (int i=0; i < list.size(); i++) {
      //println(list.get(i));
      py += textAscent()+2;
      text(list.get(i), px, py);
    }
  }
}