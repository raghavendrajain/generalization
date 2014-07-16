#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/python.hpp>
#include <Python.h>
#include <dlfcn.h>
// #include "StorePosition.cpp" 
// #include "ControlRotation.h"
#include <typeinfo>



#define PI 3.14159265


using namespace std;
namespace py = boost::python; // create namespace variable for boost::python
std::string parse_python_exception();

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt);   
 
private:
  bool accelerationValueRecieved;
  Rotation linkRotation; 
 
};  

static double xPos = 0; 
static double yPos= 0; 


static int messageCount;
static int onActionCount;
static int flag=1;
Vector3d pos;

Vector3d startPosition;

Rotation initialToolRot;

Rotation partsRot;

template <typename T> string tostr(const T& t) { ostringstream os; os<<t; return os.str(); } // template to convert double variables to string


void MyController::onInit(InitEvent &evt) {  


SimObj *stick = getObj("robot_test");
// stick->addForce(-5000,0,5000);

stick->getPosition(startPosition);
// stick->getRotation(initialToolRot);
 
}  
  
double MyController::onAction(ActionEvent &evt) {  

  dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL); 
  Py_Initialize();  //initialization of the python interpreter




  //return 1.0;
  SimObj *stick = getObj("robot_test");
  SimObj *box   = getObj("box_001");
  SimObj *goal_001   = getObj("checkpoint_001");
  double torque;





  // The target position
  Vector3d targetPos;
  box->getPosition(targetPos);

  // The Goal Position: The sphere is placed at Goal position. 

  Vector3d goalPos;
  goal_001->getPosition(goalPos);

  // calculating the displacement vector 

  Vector3d displacementVect;
  displacementVect.x(goalPos.x()-targetPos.x());
  displacementVect.y(goalPos.y()-targetPos.y());
  displacementVect.z(goalPos.z()-targetPos.z());



  // stick->addForce(-500,0,500);  // This adds force to on the stick tool.
  // stick->addForce(0,0,5000); 

  // Vector3d angularVel;
  // stick->getAngularVelocity(angularVel); 
  // LOG_MSG((" Current angular Velocity is  : %f %f %f ", angularVel.x(), angularVel.y(), angularVel.z() ));

 //  if ( abs(angularVel.y()) > 0.1  )
 //  {
 //      LOG_MSG((" Current angular Velocity is  : %f %f %f ", angularVel.x(), angularVel.y(), angularVel.z() ));
 //      double* ptr1 = NULL;
 //      ptr1 = controlAngularVelocity(angularVel, 3.0, 0, 1.0);
 //      torque  =  ptr1[0] * 500;
 //      // torque =  ptr1[0] * 12000 ;
 //      cout << "The torque applied for controlling angular velocity is " << torque << "   N. m" << endl; 
 //      stick->addTorque( 0 , torque, 0);
 // } 


  // to control the rotation of the tool

  // Rotation currentToolRot;
  // stick->getRotation(currentToolRot);
  // double *ptr = NULL;
  // ptr = controlRotation(initialToolRot, currentToolRot, 20.0, 0.0, 20.0);
  // torque  =  ptr[1] * 200 ;
  // cout << "The torque applied for controlling the rotation = " << torque << endl;
  // stick->addTorque(0, torque,0);





  // double massOfTool;  
  // massOfTool = stick->getMass();  
  // cout << "The mass is " << massOfTool <<endl;

  // Vector3d velocityOfTarget;
  // box->getLinearVelocity(velocityOfTarget);

  // double netVelocityTarget;
  // netVelocityTarget=( pow(velocityOfTarget.x(),2) + pow(velocityOfTarget.y(), 2) + pow(velocityOfTarget.z(), 2 ) );
  // netVelocityTarget = sqrt(netVelocityTarget);


  // stick->getPosition(pos);
  // stick->getPartsPosition(pos, "LINK1");
  // LOG_MSG((" LINK1 Position is  : %f %f %f ", pos.x(), pos.y(), pos.z() ));

  // Vector3d targetPos;
  // box->getPosition(targetPos);

  // Vector3d goalPos;
  // goal_001->getPosition(goalPos);

  // if (abs( goalPos.z() - targetPos.z()) < 1.4 ) 
  // {
  //    cout << "The distance to goal is " << abs( goalPos.z() - targetPos.z()) << endl;
  //    cout << "The goal has been reached " << endl;
  //    exit(0);
  // }

  // if (netVelocityTarget > 0.1 )

  // {

  //   double angle = atan ( (targetPos.z() - startPosition.z()) / (targetPos.x() - startPosition.x()  )  ) * 180 / PI;
  //   cout << "The angle is" << angle; 
  //   storePosition(targetPos);
  // }



  std::vector<std::string> s;

  for(std::map<std::string, CParts *>::iterator it = stick->getPartsCollection().begin(); it != stick->getPartsCollection().end(); ++it){
    if (it->first != "body")
        s.push_back(it->first);
    }
  
 std::string linkName; 
 Size si;

 cout << "The total links are  " << s.size() << endl;

 for (int i = 0; i < s.size(); i++){

  const char* linkName = s[i].c_str();
  CParts *link = stick->getParts(linkName);
  link->getPosition(pos);
  link->getRotation(linkRotation);

  if (link->getType() == 0){
            BoxParts* box = (BoxParts*) link;
            si = box->getSize();
            // cout <<  linkName << endl;
            cout << linkName << "  position : x = " << pos.x() << "  y = " << pos.y() << "  z = " << pos.z() << endl;
            // cout << linkName << "  size : x = " << si.x() << "  y = " << si.y() << "  z = " << si.z() << endl;
            // cout << linkName << "  Rotation: qw " << linkRotation.qw() << " qx = "<< linkRotation.qx() << " qy = "<< linkRotation.qy()
            // << " qz = "<< linkRotation.qz() << endl;

            try{  

               py::object main_module = py::import("__main__");
               py::object main_namespace = main_module.attr("__dict__");  
               main_module.attr("linkName") = linkName;
               main_module.attr("length")  = si.x();
               main_module.attr("height")  = si.y();
               main_module.attr("breadth") = si.z();
               main_module.attr("linkPos") = "[" + tostr(pos.x())+" , "+ tostr(pos.y())+ " , " + tostr(pos.z()) + "]";
               main_module.attr("rotation") = "[" + tostr(linkRotation.qw())+" , "+ tostr(linkRotation.qx())+ " , " + tostr(linkRotation.qy()) + " , " + tostr(linkRotation.qz()) +"]";
               
               // calculating the rotation of the tool.
               py::exec("import ast", main_namespace);
               py::exec("import transformations as T", main_namespace);
               py::exec("rotation = ast.literal_eval(rotation)", main_namespace);
               // py::exec("angles = T.euler_from_quaternion(rotation) ", main_namespace);
               // py::exec("print angles", main_namespace);

    
             
               py::exec("linkPos = ast.literal_eval(linkPos)", main_namespace);
               py::exec_file("vertices.py", main_namespace, main_namespace );
               py::exec("getNormals(linkName, linkPos, rotation, length, height, breadth)", main_namespace);

               main_module.attr("targetPos") = "[" + tostr(targetPos.x())+" , "+ tostr(targetPos.y())+ " , " + tostr(targetPos.z()) + "]";
               main_module.attr("displacementVect") = "[" + tostr(displacementVect.x())+" , "+ tostr(displacementVect.y())+ " , " + tostr(displacementVect.z()) + "]";
               py::exec_file("displacementVector.py", main_namespace, main_namespace );
          

            }
            catch(boost::python::error_already_set const &){
                // Parse and output the exception
                std::string perror_str = parse_python_exception();
                std::cout << "Error in Python: " << perror_str << std::endl;
            }
        }

  else if(link->getType() == 1){
            CylinderParts* cyl = (CylinderParts*) link;
            cout << "Cylinder Position : x = " << pos.x() << "  y = " << pos.y() << "  z = " << pos.z() << endl;
            cout << "Cylinder Length : length = " << cyl->getLength() << endl;
            cout << "Cylinder Radius : rad = " << cyl->getRad() << endl;
        }

  else if(link->getType() == 2){
            SphereParts* sph = (SphereParts*) link;
            cout << "Sphere Position : x = " << pos.x() << "  y = " << pos.y() << "  z = " << pos.z() << endl;
            cout << "Sphere Radius : rad = " << sph->getRad() << endl;
        }


 }


 
  // stick->getPartsPosition(pos, s[1]);
  // stick->getPartsPosition(pos, "LINK1");
  // LOG_MSG((" LINK1 Position is  : %f %f %f ", pos.x(), pos.y(), pos.z() ));

  // stick->getPartsPosition(pos, "LINK2");
  // LOG_MSG((" LINK2 Position is  : %f %f %f ", pos.x(), pos.y(), pos.z() ));


  // stick->getPartsPosition(pos, "LINK3");
  // LOG_MSG((" LINK3 Position is  : %f %f %f ", pos.x(), pos.y(), pos.z() ));


  

 

 return 0.00001;
  }


std::string msg = " ";

void MyController::onRecvMsg(RecvMsgEvent &evt)
{

  // SimObj *stick = getObj("robot_test");
  SimObj *stick = getObj("ArrowStick");


  char *all_msg = (char*)evt.getMsg();
  // std::string msg;
  msg= evt.getMsg();

  LOG_MSG((msg.c_str()));


  char xPosStr[10]=" "; 
  char yPosStr[10]=" ";
  int result=0;

  result = sscanf(msg.c_str(), "%[^','],%[^',']", xPosStr, yPosStr ); 
  xPos = atof(xPosStr);
  yPos = atof(yPosStr);

  // LOG_MSG((" Position Received by Controller : %f %f ", xPos, yPos));
  //std::cout <<"xPos is" << xPos <<std::endl;

  messageCount++;
  std::cout << "The received mesage count is" << messageCount <<std::endl;

  // stick->setPosition(pos.x() + xPos, pos.y(), pos.z() + yPos);

}


void MyController::onCollision(CollisionEvent &evt) { 

    const vector<string> & wname  = evt.getWith();       // Get a name of the other  
    const vector<string> & wparts = evt.getWithParts();  // Get a parts name of the other's collision point  
    const vector<string> & mparts = evt.getMyParts();    // Get a parts of collision point of myself  
  
    // for(int i = 0; i < wname.size(); i++)  
    //   {  
    //   // Print the name of the other  
    //   LOG_MSG(("\"%s\"", wname[i].c_str()));  
    //   LOG_MSG(("\"%s\"", wparts[i].c_str()));  
    //   LOG_MSG(("\"%s\"", mparts[i].c_str())); 

    // } 

      // LOG_MSG(("\"%s\"", myParts)); 



}



std::string parse_python_exception(){
    PyObject *type_ptr = NULL, *value_ptr = NULL, *traceback_ptr = NULL;
    // Fetch the exception info from the Python C API
    PyErr_Fetch(&type_ptr, &value_ptr, &traceback_ptr);

    // Fallback error
    std::string ret("Unfetchable Python error");
    // If the fetch got a type pointer, parse the type into the exception string
    if(type_ptr != NULL){
        py::handle<> h_type(type_ptr);
        py::str type_pstr(h_type);
        // Extract the string from the boost::python object
        py::extract<std::string> e_type_pstr(type_pstr);
        // If a valid string extraction is available, use it 
        //  otherwise use fallback
        if(e_type_pstr.check())
            ret = e_type_pstr();
        else
            ret = "Unknown exception type";
    }
    // Do the same for the exception value (the stringification of the exception)
    if(value_ptr != NULL){
        py::handle<> h_val(value_ptr);
        py::str a(h_val);
        py::extract<std::string> returned(a);
        if(returned.check())
            ret +=  ": " + returned();
        else
            ret += std::string(": Unparseable Python error: ");
    }
    // Parse lines from the traceback using the Python traceback module
    if(traceback_ptr != NULL){
        py::handle<> h_tb(traceback_ptr);
        // Load the traceback module and the format_tb function
        py::object tb(py::import("traceback"));
        py::object fmt_tb(tb.attr("format_tb"));
        // Call format_tb to get a list of traceback strings
        py::object tb_list(fmt_tb(h_tb));
        // Join the traceback strings into a single string
        py::object tb_str(py::str("\n").join(tb_list));
        // Extract the string, check the extraction, and fallback in necessary
        py::extract<std::string> returned(tb_str);
        if(returned.check())
            ret += ": " + returned();
        else
            ret += std::string(": Unparseable Python traceback");
    }
  }



extern "C" Controller * createController() {  
  return new MyController;  
}  


