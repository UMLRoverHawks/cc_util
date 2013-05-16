/* 
  This is some code needed to read in yaml hsv thresholds. 

  This code will produce a single channel mask, named “mask,” 
  which should exclude everything except rocks (and some noise) 
  when combined with an image. 

    All of the code down to the horizontal line below is needed where 
  you want the final output mask. That code calls the function 
  “getThreshold(),” found below the horizontal line. 
  “getThreshold()” does the actual yaml parsing. 

  The variable cv_in->image is not declared in this code. 
  It is the input cv::Mat grabbed from a cv_bridge. 
  You will need to provide it your own code.

  The absolute path where the input yaml is located must be given. 
  The example one is /home/adam/Documents/thresholds.yaml. 
  This should be replaced with wherever your calibration output yaml file is.

  Currently, this code only accepts one yaml file, but it can be 
  easily modified to accept multiple.
*/


// declarations
cv::Mat hsvFrameIn;
std::ifstream fin("/home/adam/Documents/thresholds.yaml",
  std::ifstream::in);
YAML::Parser parser(fin);
YAML::Node th;
parser.GetNextDocument(th);


// init hsvFrameIn to the input image in hsv color
// cv_in is the cv_bridge::CvImagePtr that you hopefully grabbed
hsvFrameIn = cv_in->image.clone();
cv::cvtColor(cv_in->image, hsvFrameIn, CV_BGR2HSV);


// declare and init final mask (mask) and individual
// colors to threshold (thresh)
cv::Mat thresh(hsvFrameIn.size(), CV_8UC3),
        mask(hsvFrameIn.size(),   CV_8UC1);


// get thresholds from yaml
for (unsigned i = 0; i < th.size(); ++i)
{
  getThreshold(th[i], hsvFrameIn, thresh);
  mask |= thresh;
} // the mask containing all 6 thresholds is complete


// how we get thresholds from yaml, via yaml-cpp
void getThreshold(const YAML::Node& node, cv::Mat& frame, cv::Mat& thresh)
{
  int h, s, v;
  node["mins"]["h"] >> h;
  node["mins"]["s"] >> s;
  node["mins"]["v"] >> v;
  cv::Scalar mins(h, s, v);
  node["maxs"]["h"] >> h;
  node["maxs"]["s"] >> s;
  node["maxs"]["v"] >> v;
  cv::Scalar maxs(h, s, v);


  cv::inRange(frame, mins, maxs, thresh);
}


// example yaml input format
// these thresholds are totally wrong, please do not use them
colors: 
  -  
    mins:
      h: 20
      s: 50
      v: 50
    maxs:
      h: 40
      s: 255
      v: 255
  -
    mins:
      h: 40
      s: 50
      v: 50
    maxs:
      h: 60
      s: 255
      v: 255
