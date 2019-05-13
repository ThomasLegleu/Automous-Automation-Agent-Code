Vec3D pointTransform(Vec3D in){
  Vec3D newX = new Vec3D(1,0,0);
  float mag = in.magnitude();        
        in.normalize();
        
        
        in.rotateX(xRot);
        in.normalize();
        in.rotateY(yRot);
        in.normalize();
        in.rotateZ(-zRot);
        in.normalize();
        in.scaleSelf(mag);
        in.addSelf(worldAdjust);
        return in;
}
