
 class attractor{
  Vec3D pos;
  
  attractor(Vec3D _pos){
    pos = _pos.copy();
  }
  
  void render(){
    fill(255,255,0);
    pushMatrix();
    translate(pos.x, pos.y, pos.z);
    box(10);
    popMatrix();
  }
 
  
}
