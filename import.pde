void stlImport(String mFile) {
  TriangleMesh tmesh = (TriangleMesh) new STLReader().loadBinary(sketchPath(mFile), STLReader.TRIANGLEMESH);
  mesh = new WETriangleMesh();
  mesh.addMesh(tmesh);
  for (Face f : mesh.getFaces ()) {
    Triangle3D newM = f.toTriangle();
    meshList.add(newM);
  }
  
//  void importPts(String file){
//  String lines[] = loadStrings(file);
//  
//  for(int i = 0; i < lines.length; i++){
//    String pos[] = split(lines[i], ',');
//    Vec3D p1 = new Vec3D(float(pos[0]), float(pos[1]), float(pos[2]));
//    color tempCol = color(float(pos[3]),float(pos[4]),float(pos[5]));
//    colPt cP = new colPt(p1, tempCol);
//    captureList.add(cP);
//  }
//}
//  
  
  for(Vertex v : mesh.getVertices()){
    if(random(30) < 20){
     float rad = random(5,10);
    Vec3D newV = new Vec3D(v.x, v.y, v.z);
    agent newA = new agent(rad, newV);
    agentList.add(newA);
    }
  }
  
  for(Vertex v : mesh.getVertices()){
    if(random(100) < 5){
     float rad = random(5,10);
    Vec3D newV = new Vec3D(v.x, v.y, v.z);
    attractor newA = new attractor(newV);
    attList.add(newA);
    }
  }  
 
}
