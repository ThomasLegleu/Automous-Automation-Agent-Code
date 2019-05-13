class robot {
  Vec3D toolPos;
  Vec3D toolAim;
  Vec3D toolRot;
  Vec3D toolUp;
  Vec3D oldPos;
  Vec3D realPos;
  Vec3D realAim;
  Vec3D realUp;
  Vec3D home;
  Vec3D centerMass;

  Vec3D j5;
  Vec3D j4;
  Vec3D j2;
  Vec3D j1;
  Vec3D j3;
  Vec3D rj5;
  Vec3D rj4;
  Vec3D rj2;
  Vec3D rj1;
  Vec3D rj3;
  Line3D[] realLines;

  Vec3D tbRot;
  Vec3D target;
  float offset;
  float offsetJ2;
  float arm1;
  float arm2;
  float toolLength;
  float alignAng;

  Vec3D origin;


  float maxRad;
  float minRad;
  float maxVel;


  ArrayList history;
  ArrayList charList;
  ArrayList rangePts;

  int charSwitch;
  boolean cumulative;
  color myCol = color(0, 0, 255);

  int modeSwitch;
  Vec3D objective;

  robot() {
    toolPos = new Vec3D(0, -850, 130);
    realPos = toolPos.copy();

    target = new Vec3D(0, -860, 130);
    origin = new Vec3D(0, 0, 0);
    toolAim = new Vec3D(0, -1, 0);
    realAim = toolAim.copy();

    toolUp = new Vec3D(0, 0, 1);
    realUp = toolUp.copy();


    minRad = 720;
    maxRad = 1650;
    home = new Vec3D(0, -850, 130);
    offset = 2.67;
    offsetJ2 = 151.65;
    toolLength = 160;
    arm1 = 825;
    arm2 = 625;
    alignAng = 0;


    history = new ArrayList();    
    charList = new ArrayList();
    charSwitch = 0;
    tbRot = new Vec3D();
    cumulative = false;
    maxVel = 20;
    oldPos = toolPos.copy();
    realLines = new Line3D[5];

    // calculate 
    calcJointPos();
    realcalcJointPos();
    modeSwitch = 0;
    objective = realPos.copy();
  }

  void protocol() {
    if (modeSwitch == 0) {
      attack();
    } else if (modeSwitch == 1) {
      apply();
      modeSwitch = 2;
    } else if (modeSwitch == 2) {
      retreat();
      modeSwitch = 0;
    }
  }

  void attack() {
    println("I attack");
    rangePts = new ArrayList();
    //////find closestAgent/////////
    agent cp = closePoint();
    
    Vec3D cp0 = (Vec3D) cp.history.get(0);
    Vec3D cp1 = seekMeshNorm(cp0);
    //set the toolAim
    Vec3D tempAim = cp1.copy();
    tempAim.scaleSelf(-1);
    tempAim = limitAim(tempAim);
    
    //ind intermediate point
    Vec3D interPos = tempAim.copy();
    interPos.scaleSelf(-450);
    interPos.addSelf(cp0);
    
    toolPos = interPos.copy();
    limitPos();
    toolAim = tempAim.copy();
    
    toolUp = new Vec3D(0,0,1);
    calcJointPos();
    
    
    streamOut(this, 1, 250);
    
    //find final tool Position
    Vec3D tempPos = tempAim.copy();
    tempPos.scaleSelf(-300);
    tempPos.addSelf(cp0);
    
    toolPos = tempPos.copy();
    limitPos();
    calcJointPos();
    
    streamOut(this,0,125);
     objective = toolPos.copy();
    modeSwitch = 1;
    
   
    
  }

  void apply() {
    println("I apply");
    agent cp = closePoint();
    
  
    
    for(int i = 0; i < cp.history.size(); i++){
      Vec3D cp0 = (Vec3D) cp.history.get(i);
      cp0 = seekMeshPt(cp0);
      Vec3D tempAim = seekMeshNorm(cp0);
     
      limitAim(tempAim);
      toolAim = tempAim.copy();
      tempAim.scaleSelf(-300);
      tempAim.addSelf(cp0);      
    
    toolPos = tempAim.copy();
    limitPos();
    calcJointPos();
    
    streamOut(this,0,250);
    }
    
    objective = toolPos.copy();
    
  }


  void retreat() {
    
    //pull back from surface
    Vec3D temp = toolAim.copy();
    temp.scaleSelf(-50);
    temp.addSelf(toolPos);
    
    toolPos = temp.copy();
    limitPos();
    calcJointPos();
    streamOut(this,0,250);
    
    //return home
    toolPos = home.copy();
  toolAim = new Vec3D(0,-1,0);
    toolUp = new Vec3D(0,0,1);
    calcJointPos();
    streamOut(this,0,250);
    
    objective = home.copy();
    //rotateMesh();
    
  }

  void calcJointPos() {
    //calc j5 pos
    Vec3D flato = new Vec3D(origin.x, origin.y, 0);

    j5 = toolAim.copy();
    j5.scaleSelf(-1*toolLength);
    j5.addSelf(toolPos);

    //calc j2
    Vec2D flatVec = new Vec2D(j5.x, j5.y);    
    Circle c = new Circle(origin.x, origin.y, offset);
    Vec2D tPts[] = c.getTangentPoints(flatVec);
    Vec3D tan1 = new Vec3D(tPts[0].x, tPts[0].y, 0);
    Vec3D tan2 = new Vec3D(tPts[1].x, tPts[1].y, 0);
    tan1.subSelf(flato);
    //tan2.subSelf(flato);
    tan1.normalize();    
    Vec3D cro = new Vec3D(j5.x, j5.y, 0);
    cro.normalize();
    cro = cro.cross(tan1);
    if (cro.z > 0) {
      tan1.scaleSelf(offset);
      tan1.z = 0;

      //tan1.addSelf(fVec);
      j2 = tan1.copy();
    } else {
      tan2.z = 0;

      //tan1.addSelf(fVec);
      j2 = tan2.copy();
    }  
    Vec3D axis = j2.sub(origin);
    Vec3D fVec = flatVec.to3DXY();
    fVec.subSelf(tan2);
    fVec.normalize();
    fVec.scaleSelf(offsetJ2);
    j2.addSelf(fVec);
    //calc j4
    float dis = j5.distanceTo(j2);
    float angC = acos((pow(arm1, 2) - pow(arm2, 2) + pow(dis, 2))/(2*arm1*dis));
    //println(angC);
    Vec3D temp = j5.copy();
    temp.subSelf(j2);
    temp.normalize();
    j4 = temp.copy();
    axis.normalize();
    j4.rotateAroundAxis(axis, -angC);
    j4.normalize();
    j4.scaleSelf(arm1);
    j4.addSelf(j2);
  }



//constrain the toolAim

  Vec3D limitAim(Vec3D in) {
   in.z = 0;
   in.normalize();
   
   Vec3D test = new Vec3D(0,-1,0);
   float ang = in.angleBetween(test);
   
   if(ang > PI/6){
     ang = ang - PI/6;
     Vec3D cro = in.cross(test);
     cro.normalize();
     in.rotateAroundAxis(cro, ang);
   }   
   return in;
  }

  void limitPos() {
    if (toolPos.z < -200) {
      toolPos.z = -200;
    }
    float dis = toolPos.distanceTo(origin);
    if (dis > maxRad) {
      toolPos.subSelf(origin);
      toolPos.limit(maxRad);
      toolPos.addSelf(origin);
    }
    Vec3D flatT = new Vec3D(toolPos.x, toolPos.y, 0);
    dis = flatT.magnitude();
    if (dis < minRad) {
      flatT.normalize();
      flatT.scaleSelf(minRad);
      toolPos.x = flatT.x;
      toolPos.y = flatT.y;
    }
  }



////////////////////////////////////////////////////////// joint movement /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void realcalcJointPos() {
    //calc j5 pos
    Vec3D flato = new Vec3D(origin.x, origin.y, 0);

    rj5 = realAim.copy();
    Vec3D rToolExtend = realAim.copy();
    rToolExtend.normalize();
    rToolExtend.scaleSelf(130);
    rToolExtend.addSelf(realPos);
    rj5.scaleSelf(-1*toolLength);
    rj5.addSelf(realPos);
    realLines[4] = new Line3D(rj5, rToolExtend);
    //calc j2
    Vec2D flatVec = new Vec2D(rj5.x, rj5.y);    
    Circle c = new Circle(origin.x, origin.y, offset);
    Vec2D tPts[] = c.getTangentPoints(flatVec);
    Vec3D tan1 = new Vec3D(tPts[0].x, tPts[0].y, 0);
    Vec3D tan2 = new Vec3D(tPts[1].x, tPts[1].y, 0);
    tan1.subSelf(flato);
    //tan2.subSelf(flato);
    tan1.normalize();    
    Vec3D cro = new Vec3D(rj5.x, rj5.y, 0);
    cro.normalize();
    cro = cro.cross(tan1);
    if (cro.z > 0) {
      tan1.scaleSelf(offset);
      tan1.z = 0;

      //tan1.addSelf(fVec);
      rj2 = tan1.copy();
    } else {
      tan2.z = 0;

      //tan1.addSelf(fVec);
      rj2 = tan2.copy();
    }  
    Vec3D axis = rj2.sub(origin);
    Vec3D fVec = flatVec.to3DXY();
    fVec.subSelf(tan2);
    fVec.normalize();
    fVec.scaleSelf(offsetJ2);
    rj2.addSelf(fVec);
    //calc j4
    float dis = rj5.distanceTo(rj2);
    float angC = acos((pow(arm1, 2) - pow(arm2, 2) + pow(dis, 2))/(2*arm1*dis));
    //println(angC);
    Vec3D temp = rj5.copy();
    temp.subSelf(rj2);
    temp.normalize();
    rj4 = temp.copy();
    axis.normalize();
    rj4.rotateAroundAxis(axis, -angC);
    rj4.normalize();
    rj4.scaleSelf(arm1);
    rj4.addSelf(rj2);
    realLines[3] = new Line3D(rj5, rj4);
    realLines[2] = new Line3D(rj4, rj2);
    realLines[1] = new Line3D(rj2, origin);
    realLines[0] = new Line3D(origin, new Vec3D(origin.x, origin.y, origin.z -32));
  }



  void updateAim() {
    Vec3D tempT = target.copy();
    tempT.subSelf(toolPos);
    tempT.normalize();
    toolAim.normalize();
    float ang = toolAim.angleBetween(tempT);
    if (ang > 0) {
      if (ang > PI/100) {
        ang = PI/100;
      }

      Vec3D cro = toolAim.cross(tempT);
      if (cro.magnitude() > 0) {
        toolUp.normalize();
        cro.normalize();
        toolAim.rotateAroundAxis(cro, ang);
        if (toolUp.angleBetween(cro) > 0) {
          toolUp.rotateAroundAxis(cro, ang);
        }
        toolAim.normalize();
        toolUp.normalize();
      }
    }
  }



  agent closePoint() {

    Vec3D closestP = home.copy();
    Vec3D closestN = toolAim.copy();
    float closeDist = 99999;
    int closeA = 0;
     
     //////find closestAgent///////
    for(int i = 0; i < agentList.size(); i++){
      agent a = (agent) agentList.get(i);
      float dis = home.distanceTo(a.pos);
      if(dis < closeDist){
        closeDist = dis;
        closeA = i;
      }
    }  
    
    agent out = (agent) agentList.get(closeA);
    
    return out;
  }



  void dummyUpdate() {
    if(dumList.size() > 0){
      dumPt d1 = (dumPt) dumList.get(0);
      Vec3D acc = d1.pos.copy();
      acc.subSelf(realPos);
      
      if(acc.magnitude() < .1){
        dumList.remove(0);
      }else{
        acc.limit(d1.speed/50);
      realPos.addSelf(acc);
      realAim = d1.aim.copy();
      realcalcJointPos();
      }

    }
  }


  void updateOrbit() {

    Vec3D temp = toolPos.sub(origin);

    temp.normalize();
    temp.rotateAroundAxis(new Vec3D(0, 0, 1), PI/180);
    float mag = sin(radians(frameCount))*.5+.5;
    mag = mag *(maxRad - minRad) + minRad;
    temp.normalize();
    Vec3D axis = temp.cross(new Vec3D(0, 0, 1));
    float vert =( sin(radians(2.3*frameCount))*150)-100;
    temp.z = 0;
    temp.normalize();

    temp.scaleSelf(mag);
    temp.addSelf(origin);
    toolPos = temp.copy();
    toolPos.z = vert;
    limitTool();
    calcJointPos();
  }  

  void limitTool() {
    float dis = toolPos.distanceTo(origin);
    if (dis > maxRad) {
      toolPos.subSelf(origin);
      toolPos.normalize();
      toolPos.scaleSelf(maxRad);
      toolPos.addSelf(origin);
    }
    Vec3D flatVec = toolPos.copy();
    flatVec.z = 0;

    if (flatVec.magnitude() < minRad) {
      flatVec.normalize();
      flatVec.scaleSelf(minRad);
      flatVec.z = toolPos.z;
      toolPos = flatVec.copy();
    }
    if (toolPos.z < -160) {
      toolPos.z = -160;
    }
  }

  void render() {
    stroke(myCol);
    noFill();
    beginShape();
    vertex(toolPos.x, toolPos.y, toolPos.z);
    vertex(j5.x, j5.y, j5.z);
    vertex(j4.x, j4.y, j4.z);
    vertex(j2.x, j2.y, j2.z);
    vertex(origin.x, origin.y, origin.z);
    vertex(0, 0, -320);
    endShape();

    stroke(150);
    noFill();
    beginShape();
    vertex(realPos.x, realPos.y, realPos.z);
    vertex(rj5.x, rj5.y, rj5.z);
    vertex(rj4.x, rj4.y, rj4.z);
    vertex(rj2.x, rj2.y, rj2.z);
    vertex(origin.x, origin.y, origin.z);
    vertex(0, 0, -320);
    endShape();

    stroke(255, 0, 0);

    Vec3D tempUp = toolUp.copy();
    tempUp.scaleSelf(10);
    tempUp.addSelf(toolPos);
    line(toolPos.x, toolPos.y, toolPos.z, tempUp.x, tempUp.y, tempUp.z);

    noStroke();
    fill(255, 0, 0);
    pushMatrix();
    translate(j5.x, j5.y, j5.z);
    sphere(20);
    popMatrix();

    noStroke();
    fill(255, 0, 0);
    pushMatrix();
    translate(j2.x, j2.y, j2.z);
    sphere(20);
    popMatrix();

    noStroke();
    fill(255, 0, 0);
    pushMatrix();
    translate(j4.x, j4.y, j4.z);
    sphere(20);
    popMatrix();

    noStroke();
    fill(255, 0, 0);
    pushMatrix();
    translate(origin.x, origin.y, origin.z);
    sphere(20);
    popMatrix();

    noStroke();
    fill(255, 0, 0);
    pushMatrix();
    translate(0, 0, 0);
    sphere(20);
    popMatrix();
    
    stroke(255, 0, 0);

    tempUp = realUp.copy();
    tempUp.scaleSelf(10);
    tempUp.addSelf(realPos);
    line(realPos.x, realPos.y, realPos.z, tempUp.x, tempUp.y, tempUp.z);

    stroke(0, 0, 255);
   
  }
}

public class Node implements Comparable {
  float d;
  Vec3D o;
  Vec3D n;

  public Node(float _dist, Vec3D _o, Vec3D _n) {
    o = _o;
    d = _dist;
    n = _n;
  }

  public int compareTo(Object n1) {
    if (d == ((Node) n1).d)
      return 0;
    else if (d > ((Node) n1).d)
      return 1;
    else
      return -1;
  }
}

