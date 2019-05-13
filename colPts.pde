class colPt{
  Vec3D pos;
  color myCol;
  
  colPt(Vec3D _pos, color _myCol){
    pos = _pos.copy();
    myCol = _myCol;
  }
  
  void render(){
    stroke(myCol);
    point(pos.x, pos.y, pos.z);
  }
  
}
