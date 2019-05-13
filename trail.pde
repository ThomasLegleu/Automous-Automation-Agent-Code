class trail {

  Vec3D pos;
  Vec3D vel;
  float life;
  agent parent;

  //this is the constructor
  trail(Vec3D _pos, Vec3D _vel, agent _parent) {

    pos = _pos.copy();
    vel = _vel.copy();
    vel.normalize();
    life = life1;
    parent = _parent;
  }


  void renderT() {
    stroke(205*life/100+100,50,50);
    Vec3D temp = vel.copy();
    temp.scaleSelf(tLength);
        if(showTrail==true) {
    line(pos.x, pos.y, pos.z,pos.x + temp.x, pos.y+ temp.y, pos.z + temp.z);
        }
   
    life = life -1;
    if(life < 0){
      trailList.remove(this);
    }
  }
}
