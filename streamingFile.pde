void streamOut(robot r, int erase, float speed) {
 
  String x = convertFloat(r.toolPos.x);
  String y = convertFloat(r.toolPos.y);
  String z = convertFloat(r.toolPos.z);
  float[] rVals = rotConversion1(r.toolAim, r.toolUp);
  String rx = convertFloat(rVals[0]);
  String ry = convertFloat(rVals[1]);
  String rz = convertFloat(rVals[2]);
  String sp = convertSpeed(speed);
  

  String out = x+","+y + ","+z+","+rx+","+ry+","+rz+","+ sp +","+ erase +",0,0,1,0,0,0,0,0";
  if(erase == 1){
    dumList = new ArrayList();
  }
  Vec3D dP = r.toolPos.copy();
  Vec3D dA = r.toolAim.copy();
  dumPt newD = new dumPt(dP, dA, speed);
  dumList.add(newD);
  
  println(out);
 
 

}

class dumPt{
   Vec3D pos;
   float speed;
   Vec3D aim;
   
   dumPt(Vec3D _pos, Vec3D _aim, float _speed){
     pos = _pos;
     speed = _speed;
     aim = _aim.copy();
   }
}

String convertSpeed(float f) {
  String s = "";
  if(abs(f) < .001){
    f = 0.0;
  }
  if (f>= 0) {
    s ="+";
  }else{
    s = "-";
  }
float bill = abs(f);
  String df = new DecimalFormat("000").format(bill);
  if(df.equals("?") || df.equals("-?")){
    df = "000";
    //println(f);
   // println(df);
  }
  
  s = s+df;
  if(s.equals("?") || s.equals("-?")){
    s = "+000";
    //println(f);
    //println(df);
  }
  //println(s);
  return s;
}


String convertFloat(float f) {
  String s = "";
  if(abs(f) < .001){
    f = 0.0;
  }
  if (f>= 0) {
    s ="+";
  }else{
    s = "-";
  }
float bill = abs(f);
  String df = new DecimalFormat("00000.000").format(bill);
  if(df.equals("?") || df.equals("-?")){
    df = "00000.000";
    //println(f);
   // println(df);
  }
  
  s = s+df;
  if(s.equals("?") || s.equals("-?")){
    s = "+00000.000";
    //println(f);
    //println(df);
  }
  //println(s);
  return s;
}
void updatePosAim(float[] angIn, Vec3D nP, robot me){
  Vec3D newZ = new Vec3D(0,0,1);
  Vec3D newX = new Vec3D(1,0,0);
  Vec3D newY = new Vec3D(0,1,0);
  newZ.rotateAroundAxis(newX, radians(angIn[0]));
  newY.rotateAroundAxis(newX, radians(angIn[0]));
  newZ.normalize();
  newY.normalize();
  newZ.rotateAroundAxis(newY, radians(angIn[1]));
  newX.rotateAroundAxis(newY, radians(angIn[1]));
  newZ.normalize();
  newX.normalize();
  newX.rotateAroundAxis(newZ,radians(angIn[2]));
  newX.normalize();
 
  me.realAim = newZ.copy();
  me.realUp = newX.copy();
  me.realPos = nP.copy();
  me.realcalcJointPos(); 
  
}

float[] rotConversion1(Vec3D aim, Vec3D orient) {
  float[] tbRot = new float[3];
  tbRot[0] = 0;
  tbRot[1] = 0;
  tbRot[2] = 0;
  aim.normalize();
  orient.normalize();  
  Vec3D toolY = aim.cross(orient);
  toolY.normalize();

  Vec3D[] targetFrame = {
    orient, toolY, aim
  };

  Vec3D[] oFrame = {
    new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1)
    };

    ////////find rotation around xAxis//////////

    float dscalar =  targetFrame[0].dot(oFrame[0]);
  if (abs(dscalar) != 1) {
    if (dscalar != 0) {
      Vec3D d1 = oFrame[0].copy();
      d1.scaleSelf(-1*dscalar);
      d1.addSelf(targetFrame[1]);  
      d1.normalize();
      float rotAng = d1.angleBetween(oFrame[1]);
      Vec3D cro = d1.cross(oFrame[1]);
      cro.normalize();
      if (cro.dot(oFrame[0]) > 0) {
        rotAng = rotAng*-1;
      }

      oFrame[1] = oFrame[1].rotateAroundAxis(oFrame[0], rotAng);
      oFrame[1].normalize();
      oFrame[2] = oFrame[2].rotateAroundAxis(oFrame[0], rotAng);
      oFrame[2].normalize();
      tbRot[0] = degrees(rotAng);
    } else {
      println("Im 90");
      float rotAng = oFrame[0].angleBetween(targetFrame[0]);
      oFrame[1] = oFrame[1].rotateAroundAxis(oFrame[0], rotAng);
      oFrame[1].normalize();
      oFrame[2] = oFrame[2].rotateAroundAxis(oFrame[0], rotAng);
      oFrame[2].normalize();
      tbRot[0] = degrees(rotAng);
    }
  } else {
    tbRot[0] = 0;
  }

  ///////find rotation around new y axis////////////
  dscalar = targetFrame[1].dot(oFrame[1]);
  if (abs(dscalar) != 1) {
    if (dscalar != 0) {
      Vec3D d1 = oFrame[1].copy();
      d1.scaleSelf(-1*dscalar);
      d1.addSelf(targetFrame[2]);
      d1.normalize();
      float rotAng = d1.angleBetween(oFrame[2]);
      Vec3D cro = d1.cross(oFrame[2]);
      cro.normalize();
      if (cro.dot(oFrame[1]) > 0) {
        rotAng = -1*rotAng;
      }
      oFrame[0] = oFrame[0].rotateAroundAxis(oFrame[1], rotAng);
      oFrame[0].normalize();
      oFrame[2] = oFrame[2].rotateAroundAxis(oFrame[1], rotAng);
      oFrame[2].normalize();
      tbRot[1] = degrees(rotAng);
    } else {
      float rotAng = oFrame[2].angleBetween(targetFrame[2]);
      oFrame[0] = oFrame[0].rotateAroundAxis(oFrame[1], rotAng);
      oFrame[0].normalize();
      oFrame[2] = oFrame[2].rotateAroundAxis(oFrame[1], rotAng);
      oFrame[2].normalize();
      tbRot[1] = degrees(rotAng);
    }
  } else {
    tbRot[1] = 0;
  }
  /////// z axis now hate euler shits///////////
  dscalar = targetFrame[2].dot(oFrame[2]);
  if (abs(dscalar) != 0) {
    Vec3D d1 = oFrame[2].copy();
    d1.scaleSelf(-1*dscalar);
    d1.addSelf(targetFrame[0]);
    d1.normalize();
    float rotAng = d1.angleBetween(oFrame[0]);
    Vec3D cro = d1.cross(oFrame[0]);
    if (cro.dot(oFrame[2]) > 0) {
      rotAng = -1*rotAng;
    }
    tbRot[2] = degrees(rotAng);
  } else {
    float rotAng = oFrame[0].angleBetween(targetFrame[0]);
    
    tbRot[2] = degrees(rotAng);
  }
  for (int i = 0; i < tbRot.length; i++) {
    if (Float.isNaN(tbRot[i] )) {
      tbRot[i] = 0;
    }
  }

  return tbRot;
}


