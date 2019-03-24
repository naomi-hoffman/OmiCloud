import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import java.util.Random;
import Jama.*; 

import java.net.URL;
import java.net.MalformedURLException;
import javax.net.ssl.HttpsURLConnection;


// Kinect Library object
Kinect kinect;

final float C_EPSILON = 0.05;
final float RANSAC_F = 0.75;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

// the URL of the Azure endpoint
URL url;
Random rand;

//the equivalent of main-method. here is where we initialize the functions we need to use.
void setup() {
  rand = new Random();
  // Rendering in P3D
  //size(800, 600, P3D);
   size(1280, 520, P3D);
  
  //create a kinect object + initialize
  kinect = new Kinect(this);
  kinect.initDepth();
  kinect.initVideo();
  
  frameRate(60);

  try {
    // put the azure endpoint here
    url = new URL("https://prod-46.eastus.logic.azure.com/workflows/6a5966fd5508414d85d083507857d9da/triggers/manual/paths/invoke?api-version=2016-10-01&sp=%2Ftriggers%2Fmanual%2Frun&sv=1.0&sig=lQeMYniSgI4cTRzfOlUsiDZ9DMHQCl-YUCkoEUo6TME");
  } catch (MalformedURLException e) {
    e.printStackTrace();
  }

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
}

// send signal to http
void tellOmi() {
  try {
    HttpsURLConnection con = (HttpsURLConnection) url.openConnection();
    con.getContent();
  } catch (IOException e) {
    e.printStackTrace();
  }
}

void draw() {

  background(0);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  //int skip = 4;

  // Translate and rotate

  //rotateY(a);
  
  // Nested for loop that initializes x and y pixels and, for those less than the
  // maximum threshold and at every skipping point, the offset is calculated to map
  // them on a plane instead of just a line
  ArrayList<PVector> points = new ArrayList<PVector>();
  for (int x = 0; x < kinect.width; x++) {
    for (int y = 0; y < kinect.height; y ++) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      points.add(v);
      
    }
  }
  
  // filter here!
  PVector[] plane = bestPlane(points);
  PVector P0 = plane[0];
  PVector n = plane[1];
  ArrayList<PVector> planePoints = findInliers(P0, n, C_EPSILON, points);
  // ArrayList<PVector> planePoints = points;
  // make decision
  // tellOmi
  
    image(kinect.getVideoImage(), 640, 0);
    //image(kinect.getDepthImage(), 0, 0);
  translate(width/4, height/2, -50);
  for (int i = 0; i< planePoints.size(); i++) {
    PVector v = planePoints.get(i);
    stroke(255);
    pushMatrix();
    // Scale up by 200
    float factor = 300;
    translate(v.x*factor, v.y*factor, factor-v.z*factor);
    // Draw a point
    point(0, 0);
    popMatrix();
  }
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Only needed to make sense of the ouput depth values from the kinect
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

// Drawing the result vector to give each point its three-dimensional space
  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

// fitMinimalPlane
PVector fitMinimalPlane (PVector P1, PVector P2, PVector P3){
PVector v21 = P2.sub(P1);
PVector v23 = P3.sub(P2);
return v21.cross(v23);
}
// Emily does math.
// findInliers
ArrayList<PVector> findInliers(PVector P0, PVector n, double e, ArrayList<PVector> pz){
  ArrayList<PVector> inliers = new ArrayList<PVector>();
for(int i=0; i < pz.size(); i++){
  PVector pi = pz.get(i);
  PVector v0 = P0.sub(pi);
  double dis = n.dot(v0);
  if(dis <= e){
    inliers.add(pi);
  }
}
return inliers;
}
//Emily does more math!
PVector[] findBestPlane (ArrayList<PVector> inliers){
int size = inliers.size();
PVector p0 = new PVector(0, 0, 0);
double[][] M = new double[size][3];
  for(int i=0; i<size; i++){
    PVector p = inliers.get(i);
    p0 = p0.add(p);
  }

  p0 = p0.div(size);
//create double array from inliers
  for(int i=0; i<size; i++){
    PVector p = inliers.get(i);
    p = p.sub(p0);
    M[i] = new double[]{p.x, p.y, p.z};
  }
  // create matrix from double array
  Matrix mat = new Matrix(M);
  //MTM
  mat = mat.transpose().times(mat);
  EigenvalueDecomposition eigs = mat.eig();
//find min eigenvalue and index
  double[] eigenvalues = eigs.getRealEigenvalues();
  double val = eigenvalues[0];
  int idx = 0;
  for(int j=1; j<eigenvalues.length; j++){
    if(eigenvalues[j]<val){
      val = eigenvalues[j];
      idx = j;
    }
  }
  //eigenvalue matrix
Matrix mat2 = eigs.getV();
//find points from min idx and make PVector
float v01 = (float)mat2.get(idx, 0);
float v02 = (float)mat2.get(idx, 1);
float v03 = (float)mat2.get(idx, 2);
PVector normalVector = new PVector(v01, v02, v03);
PVector[] ret = new PVector[] {p0, normalVector};
return ret;
}

PVector[] bestPlane(ArrayList<PVector> pointCloud){
while(true){
// Choose 3 random points
int randNum = rand.nextInt(pointCloud.size());
PVector P1 = pointCloud.get(randNum);
randNum = rand.nextInt(pointCloud.size());
PVector P2 = pointCloud.get(randNum);
randNum = rand.nextInt(pointCloud.size());
PVector P3 = pointCloud.get(randNum);

//P0 is any of P1, P2, or P3. for simplicity, choose P1
PVector n = fitMinimalPlane(P1, P2, P3);
ArrayList<PVector> inlier = findInliers ( P1, n, C_EPSILON, pointCloud);
float m = inlier.size();
float N = pointCloud.size();
if(m/N > RANSAC_F) {
  return findBestPlane(inlier);
}
}
}

//Emily is tired of math.
//A-B
//A: base
//B: current room
ArrayList<PVector> aMinusB(ArrayList<PVector> pointCloudB, ArrayList<PVector> pointCloudClean, double E){
  ArrayList<PVector> grandma = new ArrayList<PVector>();
  for(int i = 0; i < pointCloudB.size(); i++){
    //A-B
    PVector pv1 = pointCloudClean.get(i);
    PVector pv2 = pointCloudB.get(i);
    if(pv1.sub(pv2) > E){
      grandma.add(pv2);
    }
  }
  return grandma;
}

//Grandma above waterline
boolean isGrandmaDrowning(PVector[] waterline, ArrayList<PVector>() gma, double E){
//distance point plane
int belowCount = 0;
int aboveCount = 0;
//count how much of grandma is above or below the waterline
for(int i = 0; i < gma.size(); i++){
  PVector sub = gma.get(i).sub(waterline[0]);
  double dot = waterline[1].dot(sub);
  if(dot<=0){
    belowCount++;
  }
  else{
    aboveCount++;
  }
}
//ratio of point above and below
double ratio = belowCount/aboveCount;
if(ratio > E){
  return true;
}
return false;
}
