// Built with Processing and Processing.js
// https://hadamlenz.wordpress.com/2014/07/16/building-a-button-in-processing/
class Plot extends View {
  int idx, r, g, b, ploth, plotw;
  float minY, maxY;
  FloatList list;
  String label;

  String float2String(float number) {
    String s = String.format("%.2f", number);
    s = s.replaceAll(",", ".");
    return s;
  }

  Plot(View aparent, int _idx, float _minY, float _maxY, String _label, int _posX, int _posY, int _plotw, int _ploth, int _r, int _g, int _b) {
    super(aparent, _posX, _posY, _plotw, _ploth);    
    idx = _idx;
    minY = _minY;
    maxY = _maxY;
    label = _label;
    xpos = _posX;
    ypos = _posY;
    r = _r;
    g = _g;
    b = _b;
    ploth = _ploth;
    plotw = _plotw;
    list = new FloatList();
  }

  void addPlotData(float value) {  
    list.append( value );
    if (list.size() > plotw) list.remove(0);
  }

  void update() {  
    super.update();
    int px = getXPos();
    int oldpy = 0;  
    int py;
    float rangeY = abs(maxY-minY);   
    float stepY = ((float)ploth)/rangeY;  
    stroke(0, 0, 0);
    fill(255, 255, 255);
    if (idx==0) {
      rect(px, getYPos(), plotw, ploth, 10);
    }
    // zero line
    stroke(200, 200, 200);
    py = getYPos() + ploth-((int)( (0-minY) *stepY ));
    if (idx==0) {
      line(px, py, px+plotw, py);
    }
    fill(r, g, b);
    stroke(r, g, b);
    text(label, getXPos()+plotw+10, getYPos()+idx*20+15);
    if (list.size() < 2) return;
    text(float2String(list.get(list.size()-1)), getXPos()+plotw+90, getYPos()+idx*20+15);
    for (int i=0; i < list.size(); i++) {
      //println(list.get(i));
      float value = max(minY, min(maxY, list.get(i)));
      py = getYPos() + ploth-((int)( (value-minY) *stepY ));    
      if (i > 0) line(px-1, oldpy, px, py);    
      px++;
      oldpy = py;
    }
  }
}


/*
void plot(int idx, float minY, float maxY, FloatList list, String label, int posX, int posY, int r, int g, int b){  
  int h = ploth;
  int w = plotw;
  int px = posX;
  int oldpy = 0;  
  int py;
  float rangeY = abs(maxY-minY);   
  float stepY = ((float)h)/rangeY;  
  stroke(0,0,0);
  fill(255,255,255);
  rect(px, posY, w, h, 10);
  // zero line
  stroke(200,200,200);
  py = posY + h-((int)( (0-minY) *stepY ));
  line(px, py, px+w, py);
  fill(r,g,b);
  stroke(r,g,b);
  text(label, posX+w+10, posY+idx*20+15);
  if (list.size() < 2) return;
  text(float2String(list.get(list.size()-1)), posX+w+90, posY+idx*20+15);
  for (int i=0; i < list.size(); i++){
    //println(list.get(i));
    float value = max(minY, min(maxY, list.get(i)));
    py = posY + h-((int)( (value-minY) *stepY ));    
    if (i > 0) line(px-1, oldpy, px, py);    
    px++;
    oldpy = py;
  }
}

*/