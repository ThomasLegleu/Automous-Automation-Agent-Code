Vec3D seekMeshPt(Vec3D pIn) { 
    float closeDistance = 999999;
    
    Vec3D closestPt =  new Vec3D(); 
    Vec3D closePt = new Vec3D();
    int tIndex = 0;
    Vec3D tempP = new Vec3D(pIn.x, pIn.y, pIn.z);

    //loop through all those points
    for(int i = 0; i < meshList.size(); i++) {
      Triangle3D m = (Triangle3D) meshList.get(i);
      
        
      closePt = closestPointOnTriangle(m, tempP.copy());
      float d = tempP.distanceTo(closePt);   
      if( d < closeDistance  ) {//inView(closePt[i], 1.5) &&   
        closeDistance = d;
        closestPt = closePt.copy();
        
      }
     
    }

    
    // closestPt.limit(1);
    //Vec3D tempC = new Vec3D(closestPt.x, closestPt.y, closestPt.z);
    return closestPt;
  }
  
  Vec3D seekMeshNorm(Vec3D pIn) { 
    float closeDistance = 999999;
    
    Vec3D closestPt =  new Vec3D(); 
    Vec3D closePt = new Vec3D();
    int tIndex = 0;
    Vec3D tempP = new Vec3D(pIn.x, pIn.y, pIn.z);
    Vec3D closeNorm = new Vec3D();

    //loop through all those points
    for(int i = 0; i < meshList.size(); i++) {
      Triangle3D m = (Triangle3D) meshList.get(i);
      
        
      closePt = closestPointOnTriangle(m, tempP.copy());
      float d = tempP.distanceTo(closePt);   
      if( d < closeDistance  ) {//inView(closePt[i], 1.5) &&   
        closeDistance = d;
        //closestPt = closePt.copy();
        closeNorm = m.computeNormal();
      }
     
    }

    
    // closestPt.limit(1);
    Vec3D tempC = new Vec3D(closeNorm.x, closeNorm.y, closeNorm.z);
    return tempC;
  }
 
 
  boolean isInTriangle(Triangle3D t1, Vec3D p1){
    Vec3D v0 = t1.c.sub(t1.a);
    Vec3D v1 = t1.b.sub(t1.a);
    Vec3D v2 = p1.sub(t1.a);
    
    float dot00 = v0.dot(v0);
    float dot01 = v0.dot(v1);
    float dot02 = v0.dot(v2);
    float dot11 = v1.dot(v1);
    float dot12 = v1.dot(v2);
    
    float invDenom = 1/ ((dot00*dot11) - (dot01*dot01));
    float u = ((dot11*dot02) - (dot01*dot12))*invDenom;
    float v = ((dot00*dot12) - (dot01*dot02))*invDenom;
    
    if(u>0 && v>0 && (u+v) < 1){
      return true;
    }else{
      return false;
    }
    
  }
  Vec3D closestPointOnTriangle(Triangle3D t, Vec3D p) {
    //get normal and point on triangle
    Vec3D center = t.computeCentroid();
    Vec3D tnormal = t.computeNormal();
    tnormal = tnormal.normalize();
    Vec3D tempPoint = p.copy();
    Vec3D testVec = p.sub(center);
    //usedotProduct to find the point on infinite plane
    float s = tnormal.dot(testVec);
    testVec = tnormal.scaleSelf(s);
    Vec3D ptOnSurface = p.sub(testVec);
    
    

    //check if point is on surface
    if(isInTriangle(t, ptOnSurface)) {
      Vec3D kPtOnSurface = new Vec3D(ptOnSurface.x, ptOnSurface.y, ptOnSurface.z);
      return kPtOnSurface;
    }
    else {
      //find closest pt on edges
      Vec3D ptOnEdge = t.getClosestPointTo(p);
      Vec3D kPtOnSurface = new Vec3D(ptOnEdge.x, ptOnEdge.y, ptOnEdge.z);
      return kPtOnSurface;
    }
  }
