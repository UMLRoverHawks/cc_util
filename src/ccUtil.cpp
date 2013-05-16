#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream> 
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <functional>
#include <numeric>

enum yaml_io { sunny = 0, cloudy, overcast };
enum channels { _H = 0, _S, _V, _numchannels };
enum values { _max = 0, _min, _avg, _stddev, _numvalues };
enum _colors { _red = 0, _green, _blue, _purple,
	       _yellow, _orange, _numcolors };

namespace enc = sensor_msgs::image_encodings;

// to make the programmer's life easier for now
static const char WINDOW[] = "Color Calibration Utility";
static const char TOPIC[]  = "/camera1/image_raw";
static const char PATH[] = "";
static const int NUM_STDDEVS = 1;

class CCUtil
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  cv::Mat currentFrame;
  cv::Rect box;
  bool isDrawingBox, isPaused;
  std::vector<std::string> boxColors; // this is so we can undo
  std::map<std::string, std::vector<cv::Rect> > allBoxes;
  std::map<std::string, cv::Scalar> colors;
  std::string workingColor; // this ends up being a key to 
                            // both colors and allBoxes
  std::string currentCalibrationStr;
  int currentCalibration;
  // bool sunnyExists, overcastExists, cloudyExists;
  int framesToShowSaveMsg;

  public:
  CCUtil()
    : it(nh)
  {
    // ugly inits i'm sorry ;_;
    colors["YELLOW"] = cv::Scalar(0, 255, 255);
    colors["PURPLE"] = cv::Scalar(204, 0, 204);
    colors["ORANGE"] = cv::Scalar(0, 128, 255);
    colors["RED"]    = cv::Scalar(0, 0, 255);
    colors["GREEN"]  = cv::Scalar(0, 255, 0);
    colors["BLUE"]   = cv::Scalar(255, 0, 0);

    allBoxes["YELLOW"] = std::vector<cv::Rect>();
    allBoxes["PURPLE"] = std::vector<cv::Rect>();
    allBoxes["ORANGE"] = std::vector<cv::Rect>();
    allBoxes["RED"]    = std::vector<cv::Rect>();
    allBoxes["GREEN"]  = std::vector<cv::Rect>();
    allBoxes["BLUE"]   = std::vector<cv::Rect>();

    image_sub = it.subscribe(TOPIC, 1, &CCUtil::imageCb, this);

    // defaults, defaults
    cv::namedWindow(WINDOW);
    workingColor = "BLUE";
    cv::setMouseCallback(WINDOW, &mouseCbWrapper, this);
    boxColors = std::vector<std::string>();
    box = cv::Rect(-1, -1, 0, 0);
    isDrawingBox = false;
    isPaused     = false;
    // sunnyExists    = false;
    // overcastExists = false;
    // cloudyExists   = false;
    currentCalibrationStr = "sunny";
    currentCalibration    = 0;
    framesToShowSaveMsg = 0;

    printCLI();
  }

  ~CCUtil()
  {
    cv::destroyWindow(WINDOW);
  }

  //output thresholds as yaml
  void output_YAML(std::vector<std::vector<std::vector<int> > > &output,
		    int channel)
  {
    //get the path from predefined constant
    std::string path(PATH);
    
    //cv::FileStorage built-in class
    cv::FileStorage fs;

    //open the file depending on input
    switch(channel)
    {
    case sunny:
      fs.open((path + std::string("sunny.yml")).c_str(), cv::FileStorage::WRITE);
      break;
    case cloudy:
      fs.open((path + std::string("cloudy.yml")).c_str(), cv::FileStorage::WRITE);
      break;
    case overcast:
      fs.open((path + std::string("overcast.yml")).c_str(), cv::FileStorage::WRITE);
      break;
    }

    //output 'colors' sequence    
    fs << "colors" << "[";
    for (unsigned i = 0; i < output.size(); ++i) 
    {
      fs << "{";
      // map mins to
      fs << "mins" << "{";
      // h s v key value pairs
      fs << "h" << output[i][0][0];
      fs << "s" << output[i][0][1];
      fs << "v" << output[i][0][2];
      fs << "}";
      
      // map maxs to
      fs << "maxs" << "{";
      // h s v key value pairs
      fs << "h" << output[i][1][0];
      fs << "s" << output[i][1][1];
      fs << "v" << output[i][1][2];
      fs << "}";
      
      fs << "}"; 
    }
    fs << "]";
    //close file
    fs.release();
  }
  
  //convert a map of strings etc. to vector
  void cvtMapToVec(std::vector<std::vector<cv::Rect> > &output,
		   std::map<std::string, std::vector<cv::Rect> > &input)
  {
    std::map<std::string, std::vector<cv::Rect> >::iterator it;
    for (it = input.begin(); it != input.end(); ++it)
      output.push_back(it -> second);
    
    return;
  } 
  
  //find min, max, mean, stddev for all points
  void findRanges(int output[][_numchannels][_numvalues],
		   cv::Mat &image, std::vector<std::vector<cv::Rect> > &input)
  {
    //declarations    
    cv::Point3_<uchar> *p;
    int x, y;
    int hue, sat, val;
    int max_H, max_S, max_V, min_H, min_S, min_V;
    double avg_H, avg_S, avg_V;
    double stddev_H, stddev_S, stddev_V;
    cv::Mat image_hsv;
    std::vector<int> vecH, vecS, vecV;
    
    // convert to HSV
    cvtColor(image, image_hsv, CV_BGR2HSV);
    
    // for each color to be processed
    for (unsigned i = 0; i < input.size(); ++i)
    {
      max_H = max_S = max_V = 0;
      min_H = min_S = min_V = 255;
      avg_H = avg_S = avg_V = 0;
      stddev_H = stddev_S = stddev_V = 0;
      // for all rectangles
      for (unsigned j = 0; j < input[i].size(); ++j)
      {
        // cols
        for (x = input[i][j].tl().x; x < input[i][j].br().x; ++x)
        {
          // rows
          for (y = input[i][j].tl().y; y < input[i][j].br().y; ++y)
          {
            // retrieve point at x, y
            p = image_hsv.ptr<cv::Point3_<uchar> >(y, x);
            
            // retrieve hsv
            hue = p -> x;
            sat = p -> y;
            val = p -> z;
            
            // update max/min
            if (hue > max_H)
              max_H = hue;
            if (hue < min_H)
              min_H = hue;
            
            if (sat > max_S)
              max_S = sat;
            if (sat < min_S)
              min_S = sat;
            
            if (val > max_V)
              max_V = val;
            if (val < min_V)
              min_V = val;
            
            //push to temp vecs for mean/stddev calculation
            vecH.push_back(hue);
            vecS.push_back(sat);
            vecV.push_back(val);
          }
        }
      }
	
      if (input[i].size() != 0)
      {
        // calculate avgs
        avg_H = std::accumulate(vecH.begin(), vecH.end(), 0);
        avg_H /= vecH.size();
        
        avg_S = std::accumulate(vecS.begin(), vecS.end(), 0);
        avg_S /= vecS.size();
        
        avg_V = std::accumulate(vecV.begin(), vecV.end(), 0);
        avg_V /= vecV.size();
        
        //initialize other vectors for stddev
        std::vector<int> zero_mean_H(vecH);
        std::vector<int> zero_mean_S(vecS);
        std::vector<int> zero_mean_V(vecV);
        std::transform(zero_mean_H.begin(),
              	  zero_mean_H.end(), zero_mean_H.begin(),
              	  std::bind2nd(std::minus<int>(), avg_H));
        
        std::transform(zero_mean_S.begin(),
              	  zero_mean_S.end(), zero_mean_S.begin(),
              	  std::bind2nd(std::minus<int>(), avg_S));
        
        std::transform(zero_mean_V.begin(),
              	  zero_mean_V.end(), zero_mean_V.begin(),
              	  std::bind2nd(std::minus<int>(), avg_V));
        
        //calculate deviation with std, would be faster with boost
        int deviation = std::inner_product(zero_mean_H.begin(),
              			     zero_mean_H.end(),
              			     zero_mean_H.begin(), 0);
        stddev_H = std::sqrt(deviation / (vecH.size() - 1));
        
        deviation = std::inner_product(zero_mean_S.begin(),
              			 zero_mean_S.end(),
              			 zero_mean_S.begin(), 0);
        stddev_S = std::sqrt(deviation / (vecS.size() - 1));
        
        deviation = std::inner_product(zero_mean_V.begin(),
              			 zero_mean_V.end(),
              			 zero_mean_V.begin(), 0);
        stddev_V = std::sqrt(deviation / (vecV.size() - 1));
        
        output[i][_H][_max] = max_H; output[i][_H][_min] = min_H;
        output[i][_H][_avg] = static_cast<int>(avg_H); 
        output[i][_H][_stddev] = static_cast<int>(stddev_H);
        
        output[i][_S][_max] = max_S; output[i][_S][_min] = min_S;
        output[i][_S][_avg] = static_cast<int>(avg_S); 
        output[i][_S][_stddev] = static_cast<int>(stddev_S);
        
        output[i][_V][_max] = max_V; output[i][_V][_min] = min_V;
        output[i][_V][_avg] = static_cast<int>(avg_V);
        output[i][_V][_stddev] = static_cast<int>(stddev_V);

      } else {
        // flag for nonexistant color
        output[i][_H][_avg] = -1234;
      }

      //reinit vectors to empty
      vecH = std::vector<int>();
      vecS = std::vector<int>();
      vecV = std::vector<int>();
      
    }
    return;
  }

  //create thresholds  
  void createThresh(std::vector<std::vector<int> > &temp_vec,
                    std::vector<int> &temp_int,
                    int h_l, int s_l, int v_l,
                    int h_u, int s_u, int v_u)
  {
    //push back input vals for hsv mins
    temp_int.push_back(h_l);
    temp_int.push_back(s_l);
    temp_int.push_back(v_l);

    //push onto temp vec and reinit hsv vec
    temp_vec.push_back(temp_int);
    temp_int = std::vector<int>();
    
    //push back input vals for hsv maxs
    temp_int.push_back(h_u);
    temp_int.push_back(s_u);
    temp_int.push_back(v_u);

    //push onto temp vec and reinit hsv vec
    temp_vec.push_back(temp_int);
    temp_int = std::vector<int>();
 
    return;
  }
  
  // just call this to do everything, straightforward, 
  // last argument is what you want your yaml file to be
  void doAll(cv::Mat &image, std::map<std::string, 
              std::vector<cv::Rect> > &input, int channel)
  {
    // create wrapper output format
    int output[_numcolors][_numchannels][_numvalues];
    int hue_lower, sat_lower, val_lower;
    int hue_upper, sat_upper, val_upper;

    std::vector<std::vector<cv::Rect> > recVec;
    std::vector<std::vector<std::vector<int> > > output_final;
 
    cvtMapToVec(recVec, input);
    
    // call findRanges
    findRanges(output, image, recVec);
   
    for (unsigned i = 0; i < _numcolors; ++i)
    {
      // flag for color noexist
      if (output[i][_H][_avg] == -1234) continue;
      
      std::vector<std::vector<int> > temp_vec;
      std::vector<int> temp_int;
      
      //-/+ number of stddevs predefined
      hue_lower = output[i][_H][_avg] - (NUM_STDDEVS * output[i][_H][_stddev]);
      sat_lower = output[i][_S][_avg] - (NUM_STDDEVS * output[i][_S][_stddev]);
      val_lower = output[i][_V][_avg] - (NUM_STDDEVS * output[i][_V][_stddev]);
      
      hue_upper = output[i][_H][_avg] + (NUM_STDDEVS * output[i][_H][_stddev]);
      sat_upper = output[i][_S][_avg] + (NUM_STDDEVS * output[i][_S][_stddev]);
      val_upper = output[i][_V][_avg] + (NUM_STDDEVS * output[i][_V][_stddev]);
      
      // check for oob sat/val
      if (sat_upper > 255) sat_upper = 255;
      if (sat_lower < 0) sat_lower = 0;
      
      if (val_upper > 255) val_upper = 255;
      if (val_lower < 0) val_lower = 0;
      
      // check for oob hue
      if (hue_lower < 0)
      {
        // loop around, presumably (hue_lower < 0 && hue_upper > 179) == 0
        // mins
        createThresh(temp_vec, temp_int,
                      0, sat_lower, val_lower,
                      hue_upper, sat_upper, val_upper);
        
        output_final.push_back(temp_vec);
        temp_vec = std::vector<std::vector<int> >();
      
        // maxs
        createThresh(temp_vec, temp_int,
                      179 - abs(hue_lower), sat_lower, val_lower,
                      179, sat_upper, val_upper);
      
        output_final.push_back(temp_vec);
        temp_vec = std::vector<std::vector<int> >();
        
      } else if (hue_upper > 179)
      {
        // loop around, presumably (hue_lower < 0 && hue_upper > 179) == 0
        // otherwise we would have the whole color spectrum
        // mins
        createThresh(temp_vec, temp_int,
                      0, sat_lower, val_lower,
      	              hue_upper - 179, sat_upper, val_upper);
        
        output_final.push_back(temp_vec);
        temp_vec = std::vector<std::vector<int> >();
        
        // maxs
        createThresh(temp_vec, temp_int,
      	              hue_lower, sat_lower, val_lower,
      	              179, sat_upper, val_upper); 

        output_final.push_back(temp_vec);
        temp_vec = std::vector<std::vector<int> >();
      } else
      {
        createThresh(temp_vec, temp_int,
      	              hue_lower, sat_lower, val_lower,
      	              hue_upper, sat_upper, val_upper);
        
        output_final.push_back(temp_vec);
        temp_vec = std::vector<std::vector<int> >();
      }
    }
    // output_YAML
    output_YAML(output_final, channel); 
  }
  
  // updates the command line interface in the terminal window
  // CCUtil is being run from. This just prints the interface
  // again, nothing fancy.
  void printCLI()
  {
    std::cout << 
      "\ncontrols:\n\n" <<
      "  - draw a box by clicking & dragging\n\n" <<
      "  - choose a calibration\n" <<
      "      sunny:    '['\n" <<
      "      cloudy:   ']'\n" <<
      "      overcast: '\\'\n\n" <<
      "  - choose a selection color with the number keys\n" <<
      "      blue:   '1'\n" <<
      "      green:  '2'\n" <<
      "      red:    '3'\n" <<
      "      orange: '4'\n" <<
      "      purple: '5'\n" <<
      "      yellow: '6'\n\n" <<
      "  - undo a box by pressing 'u'\n\n" <<
      "  - confirm your selections by pressing 'space'\n\n" <<
      "  - exit with 'ctrl-c'\n\n" <<
      "----------------------------------------------------\n" <<
      "- current color: " << workingColor << "\n" <<
      "- now editing: " << currentCalibrationStr << "\n" <<
      "----------------------------------------------------" << 
      std::endl;
      // this is a thing that could be implemented at some point...
      // "calibration status:\n" <<
      // "  sunny: " exists("sunny")?"set":"not set" << std::endl;
  }

  void drawBoxes()
  {
    std::vector<cv::Rect>::iterator it_boxes;
    std::map<std::string, std::vector<cv::Rect> >::iterator it_vecs;

    // iterate through vectors of boxes, a vector for each color
    for (it_vecs = allBoxes.begin(); it_vecs != allBoxes.end(); ++it_vecs)
      {
      // iterate through individual boxes of a specific color
      for (it_boxes = it_vecs->second.begin(); 
           it_boxes != it_vecs->second.end(); ++it_boxes)
      {
        // create the actual rectangle object, to be displayed 
        // on the current frame in the imageCb function
        cv::rectangle(
          currentFrame,
          cv::Point(it_boxes->x, it_boxes->y),
          cv::Point(it_boxes->x + it_boxes->width,
                    it_boxes->y + it_boxes->height),
          colors[it_vecs->first]
       );
      }
    }
  }

  // puts the message "_____ calibration saved" in the bottom left
  // corner of ccUtil's opencv window
  void showSaveMsg()
  {
    cv::putText(currentFrame,
                currentCalibrationStr + " calibration saved.",
                cv::Point(20, currentFrame.rows - 20), 
                CV_FONT_HERSHEY_SIMPLEX, 
                0.8, 
                cv::Scalar(0, 0, 255));
  }

  static void mouseCbWrapper(int event, int x, int y, int flags, void *this_)
  {
    // we pass in 'this' as an optional argument to the "handler," which
    // is really the handler wrapper, which secretly and nefariously
    // uses 'this' CCUI instance to grab the true callback function
    static_cast<CCUtil*>(this_)->mouseCb(event, x, y, flags, 0);
  }

  // Let the user draw a box with the mouse.
  void mouseCb(int event, int x, int y, int flags, void *param)
  {
    switch(event) {
    case CV_EVENT_MOUSEMOVE:
      if (isDrawingBox) {
        box.width  = x - box.x;
        box.height = y - box.y;
      }
      break;

    case CV_EVENT_LBUTTONDOWN:
      isDrawingBox = true;
      box = cv::Rect(x, y, 0, 0);
      break;

    case CV_EVENT_LBUTTONUP:
      isDrawingBox = false;
      if (box.width < 0) {
        box.x += box.width;
        box.width *= -1;
      }
      if (box.height < 0) {
        box.y += box.height;
        box.height *= -1;
      }
      allBoxes[workingColor].push_back(box);
      boxColors.push_back(workingColor);
    }
  }

  // we can undo the last box drawn by popping it off one of allBoxes'
  // vectors. the vector "boxColors" exists so we know which of allBoxes'
  // vectors to pop off of. Whenever a new box is drawn, it's color is
  // pushed onto boxColors.
  void undoBox(void)
  {
    if (!boxColors.empty()) 
    {
      allBoxes[boxColors.back()].pop_back();
      boxColors.pop_back();
    }
  }

  // this is where most of the front end's work occurs
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_in;

    // try to grab an image from a topic specified in
    // the CCUtil constructor
    try
    {
      cv_in = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // get a keypress
    switch (cv::waitKey(10)) {
    case 32: // spacebar, save a calibration based on the boxes drawn
      doAll(currentFrame, allBoxes, currentCalibration);
      framesToShowSaveMsg = 10;
      allBoxes.clear();
      boxColors.clear();
      break;
    case 49: // 1, change box drawing color to BLUE
      workingColor = "BLUE";
      printCLI();
      break;
    case 50: // 2, ... GREEN
      workingColor = "GREEN";
      printCLI();
      break;
    case 51: // 3, ... RED
      workingColor = "RED";
      printCLI();
      break;
    case 52: // 4, ... ORANGE
      workingColor = "ORANGE";
      printCLI();
      break;
    case 53: // 5, ... PURPLE
      workingColor = "PURPLE";
      printCLI();
      break;
    case 54: // 6, ... YELLOW
      workingColor = "YELLOW";
      printCLI();
      break;
    case 117: // u, undo last box
      undoBox();
      break;
    case 91:  // [, edit sunny calibration
      currentCalibration = 0;
      currentCalibrationStr = "sunny";
      printCLI();
      break;
    case 93:  // ], edit cloudy calibration
      currentCalibration = 1;
      currentCalibrationStr = "cloudy";
      printCLI();
      break;
    case 92:  // \, edit overcast calibration
      currentCalibration = 2;
      currentCalibrationStr = "overcast";
      printCLI();
      break;
    }

    // update the frame for display
    currentFrame = cv_in->image.clone();

    // show "________ calibration saved."
    // framesToShowSaveMsg is initialized to 10
    if (framesToShowSaveMsg > 0)
    {
      showSaveMsg();
      framesToShowSaveMsg--;
    }

    // add the boxes the user has drawn to the current frame.
    drawBoxes();
    cv::imshow(WINDOW, currentFrame);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cc_util");
  CCUtil CCUI;
  ros::spin();
  return 0;
}
