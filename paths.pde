class path{
  Vec3D[] myPts;
  
  path(ArrayList inPts){
    myPts = new Vec3D[inPts.size()];
    for(int i = 0; i < myPts.length; i++){
      Vec3D temp = (Vec3D) inPts.get(i);
      myPts[i] = temp;
    }
  }
  
  void render(){
    stroke(155,0,0);
    noFill();
    beginShape();
    for(int i = 0; i < myPts.length; i++){
      vertex(myPts[i].x, myPts[i].y, myPts[i].z);
    }
    endShape();
  }
}
